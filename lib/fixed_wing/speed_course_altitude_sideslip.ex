defmodule ViaControllers.FixedWing.SpeedCourseAltitudeSideslip do
  require Logger
  require ViaUtils.Constants, as: VC

  defstruct [:tecs_energy, :tecs_balance, :roll_course_pid]

  @spec new(list()) :: struct()
  def new(config) do
    tecs_energy = ViaControllers.FixedWing.Tecs.Energy.new(config[:tecs_energy])
    tecs_balance = ViaControllers.FixedWing.Tecs.Balance.new(config[:tecs_balance])

    roll_course_ff_function = fn cmd, airspeed ->
      :math.atan(0.5 * cmd * airspeed / VC.gravity())
    end

    roll_course_config =
      config[:roll_course] |> Keyword.put(:ff_function, roll_course_ff_function)

    roll_course_pid = ViaControllers.Pid.new(roll_course_config)

    %ViaControllers.FixedWing.SpeedCourseAltitudeSideslip{
      tecs_energy: tecs_energy,
      tecs_balance: tecs_balance,
      roll_course_pid: roll_course_pid
    }
  end

  @spec update(struct(), map(), map(), number(), number()) :: tuple()
  def update(controller, commands, values, airspeed_mps, dt_s) do
    # Commands:
    # groundspeed_mps
    # course_rad
    # altitude_m
    # sideslip_rad

    # Values:
    # groundspeed_mps
    # course_rad
    # altitude_m
    # yaw_rad
    # vertical_velocity_mps

    # Steering
    deltayaw_rad =
      ViaUtils.Motion.turn_left_or_right_for_correction(
        commands.sideslip_rad + values.course_rad - values.yaw_rad
      )

    delta_course_cmd_rad =
      ViaUtils.Motion.turn_left_or_right_for_correction(commands.course_rad - values.course_rad)

    {roll_course_pid, roll_output_rad} =
      ViaControllers.Pid.update(
        controller.roll_course_pid,
        delta_course_cmd_rad,
        0,
        airspeed_mps,
        dt_s
      )

    # Speeds are [m/s]
    # Altitudes are [m]
    groundspeed = values.groundspeed_mps
    vv = values.vertical_velocity_mps
    # Logger.debug("vv: #{Common.Utils.eftb(vv,2)}")
    altitude = values.altitude_m
    # CMDs
    groundspeed_cmd = commands.groundspeed_mps
    alt_cmd = commands.altitude_m

    # Logger.debug("alt cmd/value/err: #{Common.Utils.eftb(alt_cmd,1)}/#{Common.Utils.eftb(altitude,1)}/#{Common.Utils.eftb(alt_cmd - altitude,1)}")
    # Energy Cals
    potential_energy = VC.gravity() * altitude
    kinetic_energy = 0.5 * groundspeed * groundspeed

    dV = ViaUtils.Math.constrain(groundspeed_cmd - groundspeed, -5.0, 5.0)
    groundspeed_sp = groundspeed_cmd
    potential_energy_sp = VC.gravity() * alt_cmd
    kinetic_energy_sp = 0.5 * groundspeed_sp * groundspeed_sp

    # Logger.info("pe/pe_sp: #{Common.Utils.eftb(potential_energy,1)}/#{Common.Utils.eftb(potential_energy_sp,1)}")

    energy = potential_energy + kinetic_energy
    energy_sp = potential_energy_sp + kinetic_energy_sp
    speed_dot_sp = dV * dt_s

    kinetic_energy_rate_sp = groundspeed * speed_dot_sp
    potential_energy_rate = vv * VC.gravity()

    # TECS calcs
    # Energy (thrust)
    energy_cmds = %{
      energy: energy_sp,
      kinetic_energy_rate: kinetic_energy_rate_sp,
      altitude_corr_m: alt_cmd - altitude,
      groundspeed_mps: groundspeed_cmd
    }

    energy_values = %{
      energy: energy,
      potential_energy_rate: potential_energy_rate,
      groundspeed_mps: groundspeed
    }

    {tecs_energy, thrust_output_scaled} =
      ViaControllers.FixedWing.Tecs.Energy.update(
        controller.tecs_energy,
        energy_cmds,
        energy_values,
        airspeed_mps,
        dt_s
      )

    # Balance (pitch)
    balance_cmds = %{
      kinetic_energy: kinetic_energy_sp,
      kinetic_energy_rate: kinetic_energy_rate_sp,
      altitude_corr: alt_cmd - altitude,
      groundspeed_mps: groundspeed_cmd
    }

    balance_values = %{
      kinetic_energy: kinetic_energy,
      potential_energy: potential_energy,
      potential_energy_rate: potential_energy_rate,
      groundspeed_mps: groundspeed
    }

    {tecs_balance, pitch_output_rad} =
      ViaControllers.FixedWing.Tecs.Balance.update(
        controller.tecs_balance,
        balance_cmds,
        balance_values,
        airspeed_mps,
        dt_s
      )

    output = %{
      roll_rad: roll_output_rad,
      pitch_rad: pitch_output_rad,
      deltayaw_rad: deltayaw_rad,
      thrust_scaled: thrust_output_scaled
    }

    controller = %{
      controller
      | tecs_energy: tecs_energy,
        tecs_balance: tecs_balance,
        roll_course_pid: roll_course_pid
    }

    {controller, output}
  end
end
