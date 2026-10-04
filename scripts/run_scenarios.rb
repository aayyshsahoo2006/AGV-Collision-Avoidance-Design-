#!/usr/bin/env ruby
# Detailed Multi-Scenario Simulation Runner for AGV Collision Avoidance

require_relative 'simulation_engine'
require 'fileutils'

FileUtils.mkdir_p("output")
log_file = File.open("output/simulation_run.log", "w")
log_file.puts("timestamp_s,agv_x,agv_y,agv_v,safety_state,min_dist_m,ttc_s,cmd_v")

puts "========================================================================="
puts " AGV COLLISION AVOIDANCE - DETAILED MULTI-SCENARIO SIMULATION RUNNER"
puts "========================================================================="

params = AGV::SafetyParams.new

# Scenario 2 Execution: Approaching Stationary Obstacle
puts "\n--- Executing Scenario: Approaching Stationary Obstacle (Corridor) ---"
agv_pose = AGV::Pose.new(0.0, 0.0, 0.0)
agv_v = 0.0
nominal_v = 1.2
dt = 0.05
sim_time = 0.0
obstacles = [AGV::Obstacle.new(1, 6.0, 0.0, 0.0, 0.0, 0.3)]

fsm_state = "NORMAL"
clear_accum = 0.0

120.times do |step|
  # Physics update
  agv_pose.position.x += agv_v * dt

  # Risk evaluation
  risk = AGV::RiskEngine.evaluate(agv_pose, agv_v, obstacles, params, sim_time)

  # State machine logic with hysteresis
  target_state = risk[:state]
  if fsm_state == "EMERGENCY_STOP"
    if !risk[:in_corridor] || risk[:min_dist] > (risk[:d_stop] + params.recovery_distance_buffer)
      clear_accum += dt
      fsm_state = target_state if clear_accum >= params.recovery_clear_time
    else
      clear_accum = 0.0
    end
  else
    fsm_state = target_state
  end

  # Motion controller velocity command
  cmd_v = case fsm_state
          when "NORMAL" then nominal_v
          when "WARNING" then nominal_v
          when "SLOWDOWN"
            span = risk[:d_slow] - risk[:d_stop]
            factor = span > 0.01 ? [0.2, [0.6, (risk[:min_dist] - risk[:d_stop]) / span].min].max : 0.5
            nominal_v * factor
          when "EMERGENCY_STOP" then 0.0
          end

  # Velocity dynamics
  if fsm_state == "EMERGENCY_STOP"
    agv_v = [0.0, agv_v - params.deceleration * dt].max
  else
    if agv_v < cmd_v
      agv_v = [cmd_v, agv_v + params.deceleration * dt].min
    elsif agv_v > cmd_v
      agv_v = [cmd_v, agv_v - params.deceleration * dt].max
    end
  end

  # Log
  log_file.printf("%.3f,%.3f,%.3f,%.3f,%s,%.3f,%.3f,%.3f\n",
                  sim_time, agv_pose.position.x, agv_pose.position.y, agv_v,
                  fsm_state,
                  risk[:min_dist].infinite? ? -1.0 : risk[:min_dist],
                  risk[:min_ttc].infinite? ? -1.0 : risk[:min_ttc],
                  cmd_v)

  if step % 10 == 0
    printf("t=%5.2fs | Pos: (%5.2f, %4.2f) | V: %4.2fm/s | State: %-14s | MinDist: %5.2fm | CmdV: %4.2fm/s\n",
           sim_time, agv_pose.position.x, agv_pose.position.y, agv_v,
           fsm_state,
           risk[:min_dist].infinite? ? -1.0 : risk[:min_dist],
           cmd_v)
  end

  sim_time += dt
end

log_file.close
puts "\nSimulation finished. Full telemetry saved to output/simulation_run.log\n"
