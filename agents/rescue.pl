%% =====================================================================
%% DALI2 Multi-Robot Search-and-Rescue (no battery / no charger)
%%
%% Three rescuer robots cooperate to locate victims in an arena and bring
%% them to a safe zone. Some victims are HEAVY and require two robots to
%% lift them simultaneously.
%%
%% Agents:
%%   coordinator  -- central task allocator, runs FIPA auction, deliberates
%%                   via internal event whether enough robots are ready for
%%                   a cooperative lift, optionally consults the AI oracle
%%   rescuer_1/2/3 -- mobile robots embodied in CoppeliaSim through the
%%                    Python bridge (LINDA channel <-> ZMQ Remote API)
%%   monitor       -- log + situational-awareness agent
%%
%% Communication:
%%   Bridge publishes sensor events to robots via LINDA (to=rescuer_i).
%%   Robots send actuation commands to virtual agent `sim` (the bridge).
%%   All inter-agent reasoning travels through the standard DALI2 LINDA
%%   star topology, so distribution / monitoring / replay are unchanged.
%%
%% Run:
%%   1. Start Redis (or `docker compose up redis`)
%%   2. Start CoppeliaSim, load the scene (see README)
%%   3. Start the Python bridge: python bridge/coppelia_bridge.py
%%   4. Start DALI2:
%%        swipl -l ../DALI2/src/server.pl -g main -- 8080 agents/rescue.pl
%% =====================================================================


%% =====================================================================
%% COORDINATOR
%% =====================================================================
%%
%% Receives victim sightings, maintains the open-task list, runs a
%% one-shot Vickrey-style auction (lowest cost wins) per task, and waits
%% for two robots to confirm readiness when the victim is heavy.
%% =====================================================================

:- agent(coordinator, [cycle(1)]).

%% Initial beliefs
believes(open_tasks([])).
believes(rescuer_pool([rescuer_1, rescuer_2, rescuer_3])).

%% --- Told rules: priority queue ---
%% Critical events outrank routine telemetry.
%% source_agent, event, priority
told(_, victim_seen(_,_,_,_),     200) :- true.
told(_, victim_rescued(_),        180) :- true.
told(_, propose(_),               150) :- true.
told(_, accept_proposal(_),       150) :- true.
told(_, reject_proposal(_),       150) :- true.
told(_, ready_to_lift(_,_),       140) :- true.
told(_, bid(_,_,_),               120) :- true.
told(_, position_update(_,_,_),    10) :- true.
told(_, heartbeat(_),               5) :- true.

%% --- Tell rules ---
tell(_, _, propose(_))              :- true.
tell(_, _, rescue_assignment(_,_,_,_)) :- true.
tell(_, _, lift_now(_))             :- true.
tell(_, _, log_event(_,_,_))        :- true.
tell(_, _, request_bid(_,_,_,_))    :- true.

%% --- Reactive rules ---

%% A rescuer reports a sighting. We add it to the open task list (idempotent).
victim_seenE(VictimId, X, Y, Weight) :>
    log("Sighting reported: ~w @(~2f,~2f) weight=~w", [VictimId, X, Y, Weight]),
    ( believes(known_victim(VictimId))
    ->  log("Already known: ~w", [VictimId])
    ;   assert_belief(known_victim(VictimId)),
        assert_belief(victim_info(VictimId, X, Y, Weight)),
        believes(open_tasks(L)),
        retract_belief(open_tasks(L)),
        assert_belief(open_tasks([VictimId | L])),
        send(monitor, log_event(new_task, coordinator, [VictimId, Weight])),
        %% Optional AI triage: ask the oracle to score urgency.
        ( ai_available
        ->  ask_ai(triage(victim(VictimId), location(X,Y), weight(Weight)), Advice),
            log("AI triage advice: ~w", [Advice])
        ;   true
        )
    ).

%% A robot answers an auction with its bid (lower=better).
bidE(VictimId, Robot, Cost) :>
    log("Bid received: ~w from ~w cost=~w", [VictimId, Robot, Cost]),
    assert_belief(bid(VictimId, Robot, Cost)).

%% A rescuer signals it is in position for a heavy lift.
ready_to_liftE(Robot, VictimId) :>
    log("Ready-to-lift: ~w on ~w", [Robot, VictimId]),
    assert_belief(ready_at(Robot, VictimId)),
    assert_belief(ready_at_t(Robot, VictimId)).
%% Readiness is stored as a belief; sync_liftI checks both partners
%% have the belief before issuing lift_now commands.

%% A rescuer reports the victim was delivered.
victim_rescuedE(VictimId) :>
    log("Victim ~w delivered to safe zone", [VictimId]),
    %% Cleanup
    ( retract_belief(victim_info(VictimId, _, _, _)) ; true ),
    ( retract_belief(assigned(VictimId, _)) ; true ),
    ( retract_belief(assigned_pair(VictimId, _, _)) ; true ),
    believes(open_tasks(L0)),
    retract_belief(open_tasks(L0)),
    delete(L0, VictimId, L1),
    assert_belief(open_tasks(L1)),
    send(monitor, log_event(rescue_complete, coordinator, [VictimId])).

%% Periodic robot telemetry (low-priority).
position_updateE(_Robot, _X, _Y) :> true.
heartbeatE(_Robot) :> true.

%% --- Internal events: deliberation ---

%% Auction loop: for each open task that has no assignment yet, broadcast
%% a request-for-bid. Bids are stored as beliefs and resolved one task per
%% cycle by `award_taskI`.
solicit_bidsI :>
    believes(open_tasks(Tasks)),
    member(Vid, Tasks),
    \+ believes(assigned(Vid, _)),
    \+ believes(assigned_pair(Vid, _, _)),
    \+ believes(soliciting(Vid)),
    believes(victim_info(Vid, X, Y, W)),
    log("Soliciting bids for ~w (weight=~w)", [Vid, W]),
    assert_belief(soliciting(Vid)),
    broadcast(request_bid(Vid, X, Y, W)).
internal_event(solicit_bids, 2, forever, true, forever).

%% Award task to lowest-cost bidder (light victim) or two lowest (heavy).
award_taskI :>
    believes(soliciting(Vid)),
    \+ believes(assigned(Vid, _)),
    \+ believes(assigned_pair(Vid, _, _)),
    believes(victim_info(Vid, X, Y, Weight)),
    findall(Cost-R,
            (believes(bid(Vid, R, Cost)),
             \+ believes(assigned(_, R)),
             \+ (believes(assigned_pair(_, R, _))),
             \+ (believes(assigned_pair(_, _, R)))),
            Bids),
    sort(Bids, Sorted),
    helper(award(Vid, Weight, Sorted, X, Y)).
internal_event(award_task, 1, forever, true, forever).

%% Single-clause helper with if-then-else: the loader stores the FIRST
%% matching clause and does not backtrack, so we dispatch internally.
helper(award(Vid, Weight, Bids, X, Y)) :-
    ( Bids == []
    ->  true   %% no usable bids yet -- wait for next solicitation
    ; Weight == light, Bids = [_-Robot | _]
    ->  log("Awarding ~w to ~w (light)", [Vid, Robot]),
        assert_belief(assigned(Vid, Robot)),
        send(Robot, rescue_assignment(Vid, X, Y, light)),
        helper(cleanup_bids(Vid))
    ; Weight == heavy, Bids = [_-R1, _-R2 | _], R1 \== R2
    ->  log("Awarding ~w to pair (~w, ~w) (heavy)", [Vid, R1, R2]),
        assert_belief(assigned_pair(Vid, R1, R2)),
        send(R1, rescue_assignment(Vid, X, Y, heavy(partner(R2)))),
        send(R2, rescue_assignment(Vid, X, Y, heavy(partner(R1)))),
        helper(cleanup_bids(Vid))
    ;   true   %% heavy with only one bidder -- keep waiting
    ).

helper(cleanup_bids(Vid)) :-
    retract_belief(bid(Vid, _, _)).

%% Cooperative-lift trigger: both partners are ready within delta-t (=5s,
%% enforced via past_event(ready_at_t/2,5)).
sync_liftI :>
    believes(assigned_pair(Vid, R1, R2)),
    believes(ready_at_t(R1, Vid)),
    believes(ready_at_t(R2, Vid)),
    log("COOPERATIVE LIFT triggered for ~w by ~w + ~w", [Vid, R1, R2]),
    send(R1, lift_now(Vid)),
    send(R2, lift_now(Vid)),
    %% Drop confirmations so we don't re-fire.
    ( retract_belief(ready_at(R1, Vid)) ; true ),
    ( retract_belief(ready_at(R2, Vid)) ; true ),
    ( retract_belief(ready_at_t(R1, Vid)) ; true ),
    ( retract_belief(ready_at_t(R2, Vid)) ; true ),
    send(monitor, log_event(coop_lift, coordinator, [Vid, R1, R2])).
internal_event(sync_lift, 0, forever, true, forever).

%% Dedup: known_victim is stored as a belief (not an event), so the
%% check in victim_seenE uses believes(known_victim(_)) to skip duplicates.


%% =====================================================================
%% RESCUER TEMPLATE  -- three near-identical agents follow.
%% =====================================================================

%% --------------------------- rescuer_1 -------------------------------
:- agent(rescuer_1, [cycle(1)]).

believes(robot_id(rescuer_1)).
believes(state(idle)).

%% Common rescuer rules ---------------------------------------------------

%% Sensor events from the bridge.
positionE(X, Y, Th) :>
    retract_belief(pose(_,_,_)),
    assert_belief(pose(X, Y, Th)),
    believes(robot_id(Me)),
    send(coordinator, position_update(Me, X, Y)).

at_targetE :>
    log("At target"),
    assert_belief(reached_target),
    helper(on_arrive).

obstacle_detectedE(_D) :> true.

victim_in_rangeE(Id, X, Y, Weight) :>
    believes(robot_id(Me)),
    log("Spotted victim ~w @(~2f,~2f) weight=~w", [Id, X, Y, Weight]),
    send(coordinator, victim_seen(Id, X, Y, Weight)).

deliveredE(Id) :>
    believes(robot_id(Me)),
    log("Delivered ~w -- resuming patrol", [Id]),
    ( retract_belief(carrying(_)) ; true ),
    ( retract_belief(target(Id, _, _, _)) ; true ),
    ( retract_belief(state(_)) ; true ),
    assert_belief(state(exploring)),
    send(coordinator, victim_rescued(Id)),
    send(sim, explore(Me)).

%% --- Vision events (from bridge camera + LLM analysis) ---

screenshotE(ImagePath) :>
    believes(robot_id(Me)),
    log("Screenshot captured: ~w", [ImagePath]),
    assert_belief(last_screenshot(ImagePath)).
    %% Vision analysis is handled by the bridge in a background thread;
    %% the result arrives as a vision_analysis(Desc) event.

vision_analysisE(Description) :>
    believes(robot_id(Me)),
    log("Bridge vision analysis: ~w", [Description]),
    assert_belief(last_vision_analysis(Description)).

%% --- Structured vision results (the robot acts on these) ---

vision_resultE(victim(Type)) :>
    believes(robot_id(Me)),
    believes(pose(X, Y, Th)),
    log("Vision detected ~w victim ahead", [Type]),
    %% Estimate victim position ~2m ahead of robot
    Vx is X + 2.0 * cos(Th),
    Vy is Y + 2.0 * sin(Th),
    send(coordinator, victim_seen(vision_detect, Vx, Vy, Type)),
    ( believes(state(exploring))
    ->  retract_belief(state(_)),
        assert_belief(state(idle))
    ;   true
    ).

vision_resultE(obstacle) :>
    believes(robot_id(Me)),
    log("Vision: obstacle ahead -- avoiding"),
    send(sim, avoid_obstacle(Me)).

vision_resultE(clear) :>
    log("Vision: path clear").

%% --- Exploration: patrol the arena when idle ---

exploreI :>
    believes(state(idle)),
    believes(robot_id(Me)),
    log("Starting exploration patrol"),
    retract_belief(state(_)),
    assert_belief(state(exploring)),
    send(sim, explore(Me)).
internal_event(explore, 5, forever, believes(state(idle)), forever).

%% --- Auction handling ---

request_bidE(Vid, X, Y, _Weight) :>
    %% Compute distance-based cost.
    believes(robot_id(Me)),
    ( ( believes(state(idle)) ; believes(state(exploring)) )
    ->  ( believes(pose(Px, Py, _))
        ->  Dx is Px - X, Dy is Py - Y, D is sqrt(Dx*Dx + Dy*Dy)
        ;   D = 999
        ),
        Cost = D,
        send(coordinator, bid(Vid, Me, Cost)),
        log("Bidding on ~w cost=~2f", [Vid, Cost])
    ;   log("Skipping bid on ~w (busy)", [Vid])
    ).

rescue_assignmentE(Vid, X, Y, light) :>
    believes(robot_id(Me)),
    log("Assigned: ~w (~2f,~2f) light", [Vid, X, Y]),
    retract_belief(state(_)),
    assert_belief(state(en_route_to_victim(Vid))),
    assert_belief(target(Vid, X, Y, light)),
    send(sim, set_target(Me, X, Y)).

rescue_assignmentE(Vid, X, Y, heavy(partner(Partner))) :>
    believes(robot_id(Me)),
    log("Assigned: ~w (~2f,~2f) HEAVY with ~w", [Vid, X, Y, Partner]),
    retract_belief(state(_)),
    assert_belief(state(en_route_to_victim(Vid))),
    assert_belief(target(Vid, X, Y, heavy(Partner))),
    send(sim, set_target(Me, X, Y)).

lift_nowE(Vid) :>
    believes(robot_id(Me)),
    log("Lift command received for ~w", [Vid]),
    send(sim, attach(Me, Vid)),
    %% Now drive to depot (bridge knows the depot location).
    retract_belief(state(_)),
    assert_belief(state(carrying(Vid))),
    assert_belief(carrying(Vid)),
    send(sim, go_to_depot(Me)).

%% Helper: behaviour when target is reached.
helper(on_arrive) :-
    believes(robot_id(Me)),
    ( believes(carrying(Vid))
    ->  %% Arrived at depot.
        send(sim, release(Me, Vid))
    ; believes(target(Vid, _X, _Y, light))
    ->  %% Arrived at a light victim -- pick up immediately.
        send(sim, attach(Me, Vid)),
        retract_belief(state(_)),
        assert_belief(state(carrying(Vid))),
        assert_belief(carrying(Vid)),
        send(sim, go_to_depot(Me))
    ; believes(target(Vid, _X, _Y, heavy(Partner)))
    ->  %% Arrived at a heavy victim -- wait for partner.
        log("In position for heavy lift on ~w; waiting for ~w", [Vid, Partner]),
        send(coordinator, ready_to_lift(Me, Vid))
    ; believes(state(exploring))
    ->  %% Reached exploration waypoint -- wait for vision analysis.
        log("Exploration waypoint reached -- awaiting vision"),
        retract_belief(state(_)),
        assert_belief(state(idle))
    ;   true
    ).

%% Common told/tell.
told(_, request_bid(_,_,_,_),       100) :- true.
told(_, rescue_assignment(_,_,_,_), 200) :- ( believes(state(idle)) ; believes(state(exploring)) ).
told(_, rescue_assignment(_,_,_,_),  50).        %% still queue if busy
told(_, lift_now(_),                 200) :- true.
told(_, position(_,_,_),               5) :- true.
told(_, at_target,                    50) :- true.
told(_, obstacle_detected(_),         60) :- true.
told(_, victim_in_range(_,_,_,_),    150) :- true.
told(_, delivered(_),                170) :- true.
told(_, screenshot(_),                30) :- true.
told(_, vision_analysis(_),           80) :- true.
told(_, vision_result(_),             90) :- true.

tell(_, _, bid(_,_,_))            :- true.
tell(_, _, ready_to_lift(_,_))    :- true.
tell(_, _, victim_seen(_,_,_,_))  :- true.
tell(_, _, victim_rescued(_))     :- true.
tell(_, _, position_update(_,_,_)) :- true.
tell(_, _, set_target(_,_,_))     :- true.
tell(_, _, attach(_,_))           :- true.
tell(_, _, release(_,_))          :- true.
tell(_, _, go_to_depot(_))        :- true.
tell(_, _, explore(_))            :- true.
tell(_, _, avoid_obstacle(_))     :- true.

%% Helper: process vision LLM result and act on it
helper(process_vision(victim_detected(Type))) :-
    believes(robot_id(Me)),
    believes(pose(X, Y, _)),
    log("Vision detected ~w victim near (~2f,~2f)", [Type, X, Y]),
    send(coordinator, victim_seen(vision_detect, X, Y, Type)).
helper(process_vision(obstacle_ahead)) :-
    believes(robot_id(Me)),
    log("Vision: obstacle ahead -- slowing down").
helper(process_vision(clear_path)) :-
    log("Vision: path is clear").
helper(process_vision(_Other)) :-
    true.  %% Unknown result, ignore

%% --------------------------- rescuer_2 -------------------------------
:- agent(rescuer_2, [cycle(1)]).

believes(robot_id(rescuer_2)).
believes(state(idle)).

positionE(X, Y, Th) :>
    retract_belief(pose(_,_,_)),
    assert_belief(pose(X, Y, Th)),
    believes(robot_id(Me)),
    send(coordinator, position_update(Me, X, Y)).
at_targetE :> assert_belief(reached_target), helper(on_arrive).
obstacle_detectedE(_D) :> true.
victim_in_rangeE(Id, X, Y, Weight) :>
    send(coordinator, victim_seen(Id, X, Y, Weight)).
deliveredE(Id) :>
    believes(robot_id(Me)),
    log("Delivered ~w -- resuming patrol", [Id]),
    ( retract_belief(carrying(_)) ; true ),
    ( retract_belief(target(Id, _, _, _)) ; true ),
    ( retract_belief(state(_)) ; true ),
    assert_belief(state(exploring)),
    send(coordinator, victim_rescued(Id)),
    send(sim, explore(Me)).
screenshotE(ImagePath) :>
    believes(robot_id(Me)),
    log("Screenshot captured: ~w", [ImagePath]),
    assert_belief(last_screenshot(ImagePath)).
vision_analysisE(Desc) :>
    log("Bridge vision: ~w", [Desc]),
    assert_belief(last_vision_analysis(Desc)).
vision_resultE(victim(Type)) :>
    believes(robot_id(Me)), believes(pose(X, Y, Th)),
    log("Vision detected ~w victim ahead", [Type]),
    Vx is X + 2.0 * cos(Th), Vy is Y + 2.0 * sin(Th),
    send(coordinator, victim_seen(vision_detect, Vx, Vy, Type)),
    ( believes(state(exploring)) -> retract_belief(state(_)),
        assert_belief(state(idle)) ; true ).
vision_resultE(obstacle) :>
    believes(robot_id(Me)),
    log("Vision: obstacle ahead -- avoiding"),
    send(sim, avoid_obstacle(Me)).
vision_resultE(clear) :>
    log("Vision: path clear").
exploreI :>
    believes(state(idle)), believes(robot_id(Me)),
    retract_belief(state(_)), assert_belief(state(exploring)),
    send(sim, explore(Me)).
internal_event(explore, 5, forever, believes(state(idle)), forever).
request_bidE(Vid, X, Y, _Weight) :>
    believes(robot_id(Me)),
    ( ( believes(state(idle)) ; believes(state(exploring)) ) ->
        ( believes(pose(Px,Py,_)) ->
            Dx is Px-X, Dy is Py-Y, D is sqrt(Dx*Dx+Dy*Dy)
        ; D=999 ),
        Cost = D,
        send(coordinator, bid(Vid, Me, Cost))
    ; true ).
rescue_assignmentE(Vid, X, Y, light) :>
    believes(robot_id(Me)),
    retract_belief(state(_)),
    assert_belief(state(en_route_to_victim(Vid))),
    assert_belief(target(Vid, X, Y, light)),
    send(sim, set_target(Me, X, Y)).
rescue_assignmentE(Vid, X, Y, heavy(partner(P))) :>
    believes(robot_id(Me)),
    retract_belief(state(_)),
    assert_belief(state(en_route_to_victim(Vid))),
    assert_belief(target(Vid, X, Y, heavy(P))),
    send(sim, set_target(Me, X, Y)).
lift_nowE(Vid) :>
    believes(robot_id(Me)),
    send(sim, attach(Me, Vid)),
    retract_belief(state(_)),
    assert_belief(state(carrying(Vid))),
    assert_belief(carrying(Vid)),
    send(sim, go_to_depot(Me)).
helper(on_arrive) :-
    believes(robot_id(Me)),
    ( believes(carrying(Vid)) -> send(sim, release(Me, Vid))
    ; believes(target(Vid,_,_,light)) ->
        send(sim, attach(Me, Vid)),
        retract_belief(state(_)),
        assert_belief(state(carrying(Vid))),
        assert_belief(carrying(Vid)),
        send(sim, go_to_depot(Me))
    ; believes(target(Vid,_,_,heavy(_))) ->
        send(coordinator, ready_to_lift(Me, Vid))
    ; believes(state(exploring)) ->
        retract_belief(state(_)), assert_belief(state(idle))
    ; true ).
told(_, request_bid(_,_,_,_),       100) :- true.
told(_, rescue_assignment(_,_,_,_), 200) :- ( believes(state(idle)) ; believes(state(exploring)) ).
told(_, rescue_assignment(_,_,_,_),  50).
told(_, lift_now(_),                200) :- true.
told(_, position(_,_,_),              5) :- true.
told(_, at_target,                   50) :- true.
told(_, obstacle_detected(_),        60) :- true.
told(_, victim_in_range(_,_,_,_),   150) :- true.
told(_, delivered(_),               170) :- true.
told(_, screenshot(_),               30) :- true.
told(_, vision_analysis(_),          80) :- true.
told(_, vision_result(_),            90) :- true.

tell(_, _, bid(_,_,_))            :- true.
tell(_, _, ready_to_lift(_,_))    :- true.
tell(_, _, victim_seen(_,_,_,_))  :- true.
tell(_, _, victim_rescued(_))     :- true.
tell(_, _, position_update(_,_,_)) :- true.
tell(_, _, set_target(_,_,_))     :- true.
tell(_, _, attach(_,_))           :- true.
tell(_, _, release(_,_))          :- true.
tell(_, _, go_to_depot(_))        :- true.
tell(_, _, explore(_))            :- true.
tell(_, _, avoid_obstacle(_))     :- true.

helper(process_vision(victim_detected(Type))) :-
    believes(robot_id(Me)), believes(pose(X, Y, _)),
    send(coordinator, victim_seen(vision_detect, X, Y, Type)).
helper(process_vision(obstacle_ahead)) :- true.
helper(process_vision(clear_path)) :- true.
helper(process_vision(_)) :- true.

%% --------------------------- rescuer_3 -------------------------------
:- agent(rescuer_3, [cycle(1)]).

believes(robot_id(rescuer_3)).
believes(state(idle)).

positionE(X, Y, Th) :>
    retract_belief(pose(_,_,_)),
    assert_belief(pose(X, Y, Th)),
    believes(robot_id(Me)),
    send(coordinator, position_update(Me, X, Y)).
at_targetE :> assert_belief(reached_target), helper(on_arrive).
obstacle_detectedE(_D) :> true.
victim_in_rangeE(Id, X, Y, Weight) :>
    send(coordinator, victim_seen(Id, X, Y, Weight)).
deliveredE(Id) :>
    believes(robot_id(Me)),
    log("Delivered ~w -- resuming patrol", [Id]),
    ( retract_belief(carrying(_)) ; true ),
    ( retract_belief(target(Id, _, _, _)) ; true ),
    ( retract_belief(state(_)) ; true ),
    assert_belief(state(exploring)),
    send(coordinator, victim_rescued(Id)),
    send(sim, explore(Me)).
screenshotE(ImagePath) :>
    believes(robot_id(Me)),
    log("Screenshot captured: ~w", [ImagePath]),
    assert_belief(last_screenshot(ImagePath)).
vision_analysisE(Desc) :>
    log("Bridge vision: ~w", [Desc]),
    assert_belief(last_vision_analysis(Desc)).
vision_resultE(victim(Type)) :>
    believes(robot_id(Me)), believes(pose(X, Y, Th)),
    log("Vision detected ~w victim ahead", [Type]),
    Vx is X + 2.0 * cos(Th), Vy is Y + 2.0 * sin(Th),
    send(coordinator, victim_seen(vision_detect, Vx, Vy, Type)),
    ( believes(state(exploring)) -> retract_belief(state(_)),
        assert_belief(state(idle)) ; true ).
vision_resultE(obstacle) :>
    believes(robot_id(Me)),
    log("Vision: obstacle ahead -- avoiding"),
    send(sim, avoid_obstacle(Me)).
vision_resultE(clear) :>
    log("Vision: path clear").
exploreI :>
    believes(state(idle)), believes(robot_id(Me)),
    retract_belief(state(_)), assert_belief(state(exploring)),
    send(sim, explore(Me)).
internal_event(explore, 5, forever, believes(state(idle)), forever).
request_bidE(Vid, X, Y, _Weight) :>
    believes(robot_id(Me)),
    ( ( believes(state(idle)) ; believes(state(exploring)) ) ->
        ( believes(pose(Px,Py,_)) ->
            Dx is Px-X, Dy is Py-Y, D is sqrt(Dx*Dx+Dy*Dy)
        ; D=999 ),
        Cost = D,
        send(coordinator, bid(Vid, Me, Cost))
    ; true ).
rescue_assignmentE(Vid, X, Y, light) :>
    believes(robot_id(Me)),
    retract_belief(state(_)),
    assert_belief(state(en_route_to_victim(Vid))),
    assert_belief(target(Vid, X, Y, light)),
    send(sim, set_target(Me, X, Y)).
rescue_assignmentE(Vid, X, Y, heavy(partner(P))) :>
    believes(robot_id(Me)),
    retract_belief(state(_)),
    assert_belief(state(en_route_to_victim(Vid))),
    assert_belief(target(Vid, X, Y, heavy(P))),
    send(sim, set_target(Me, X, Y)).
lift_nowE(Vid) :>
    believes(robot_id(Me)),
    send(sim, attach(Me, Vid)),
    retract_belief(state(_)),
    assert_belief(state(carrying(Vid))),
    assert_belief(carrying(Vid)),
    send(sim, go_to_depot(Me)).
helper(on_arrive) :-
    believes(robot_id(Me)),
    ( believes(carrying(Vid)) -> send(sim, release(Me, Vid))
    ; believes(target(Vid,_,_,light)) ->
        send(sim, attach(Me, Vid)),
        retract_belief(state(_)),
        assert_belief(state(carrying(Vid))),
        assert_belief(carrying(Vid)),
        send(sim, go_to_depot(Me))
    ; believes(target(Vid,_,_,heavy(_))) ->
        send(coordinator, ready_to_lift(Me, Vid))
    ; believes(state(exploring)) ->
        retract_belief(state(_)), assert_belief(state(idle))
    ; true ).
told(_, request_bid(_,_,_,_),       100) :- true.
told(_, rescue_assignment(_,_,_,_), 200) :- ( believes(state(idle)) ; believes(state(exploring)) ).
told(_, rescue_assignment(_,_,_,_),  50).
told(_, lift_now(_),                200) :- true.
told(_, position(_,_,_),              5) :- true.
told(_, at_target,                   50) :- true.
told(_, obstacle_detected(_),        60) :- true.
told(_, victim_in_range(_,_,_,_),   150) :- true.
told(_, delivered(_),               170) :- true.
told(_, screenshot(_),               30) :- true.
told(_, vision_analysis(_),          80) :- true.
told(_, vision_result(_),            90) :- true.

tell(_, _, bid(_,_,_))            :- true.
tell(_, _, ready_to_lift(_,_))    :- true.
tell(_, _, victim_seen(_,_,_,_))  :- true.
tell(_, _, victim_rescued(_))     :- true.
tell(_, _, position_update(_,_,_)) :- true.
tell(_, _, set_target(_,_,_))     :- true.
tell(_, _, attach(_,_))           :- true.
tell(_, _, release(_,_))          :- true.
tell(_, _, go_to_depot(_))        :- true.
tell(_, _, explore(_))            :- true.
tell(_, _, avoid_obstacle(_))     :- true.

helper(process_vision(victim_detected(Type))) :-
    believes(robot_id(Me)), believes(pose(X, Y, _)),
    send(coordinator, victim_seen(vision_detect, X, Y, Type)).
helper(process_vision(obstacle_ahead)) :- true.
helper(process_vision(clear_path)) :- true.
helper(process_vision(_)) :- true.


%% =====================================================================
%% MONITOR -- centralised log + situational awareness.
%% =====================================================================

:- agent(monitor, [cycle(2)]).

log_eventE(Type, Source, Data) :>
    log("EVT[~w] ~w: ~w", [Type, Source, Data]).

%% Periodic mission summary.
summaryI :>
    log("Mission heartbeat -- monitor alive").
internal_event(summary, 30, forever, true, forever).

told(_, log_event(_,_,_),      50) :- true.
told(_, request_bid(_,_,_,_),   5) :- true.   %% accept broadcasts silently