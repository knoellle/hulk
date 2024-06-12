function spawn_robot(number)
  table.insert(state.robots, create_robot(number))
end

spawn_robot(7)

state.game_controller_state.game_state = "Playing"

function on_cycle()
  if state.cycle_count == 1 then
    state.ball = {
      position = { 2.0, 0.0 },
      velocity = { 0.0, 0.0 },
    }
    set_robot_pose(7, { -4.0, 0 }, 0)

    create_obstacle(7, {-1.0, 0.0 },  0.8)
    create_obstacle(7, { 0.0, 0.0 },  1.1)
    create_obstacle(7, { 2.0, 0.0 },  0.1)
    create_obstacle(7, {-1.0, 0.0 },  0.8)
    create_obstacle(7, { 0.0, 0.0 },  1.1)
    create_obstacle(7, { 2.0, 0.0 },  0.1)
    create_obstacle(7, {-1.0, 0.0 },  0.8)
    create_obstacle(7, { 0.0, 0.0 },  1.1)
    create_obstacle(7, { 2.0, 0.0 },  0.1)
    create_obstacle(7, {-1.0, 0.0 },  0.8)
    create_obstacle(7, { 0.0, 0.0 },  1.1)
    create_obstacle(7, { 2.0, 0.0 },  0.1)
    create_obstacle(7, {-1.0, 0.0 },  0.8)
    create_obstacle(7, { 0.0, 0.0 },  1.1)
    create_obstacle(7, { 2.0, 0.0 },  0.1)
    create_obstacle(7, {-1.0, 0.0 },  0.8)
    create_obstacle(7, { 0.0, 0.0 },  1.1)
    create_obstacle(7, { 2.0, 0.0 },  0.1)
    create_obstacle(7, {-1.0, 0.0 },  0.8)
    create_obstacle(7, { 0.0, 0.0 },  1.1)
    create_obstacle(7, { 2.0, 0.0 },  0.1)
    create_obstacle(7, {-1.0, 0.0 },  0.8)
    create_obstacle(7, { 0.0, 0.0 },  1.1)
    create_obstacle(7, { 2.0, 0.0 },  0.1)
    create_obstacle(7, {-1.0, 0.0 },  0.8)
    create_obstacle(7, { 0.0, 0.0 },  1.1)
    create_obstacle(7, { 2.0, 0.0 },  0.1)
    create_obstacle(7, {-1.0, 0.0 },  0.8)
    create_obstacle(7, { 0.0, 0.0 },  1.1)
    create_obstacle(7, { 2.0, 0.0 },  0.1)
    create_obstacle(7, {-1.0, 0.0 },  0.8)
    create_obstacle(7, { 0.0, 0.0 },  1.1)
    create_obstacle(7, { 2.0, 0.0 },  0.1)
    create_obstacle(7, {-1.0, 0.0 },  0.8)
  end

  if state.cycle_count == 5000 then
    state.finished = true
  end
end

function on_goal()
  state.finished = true
end
