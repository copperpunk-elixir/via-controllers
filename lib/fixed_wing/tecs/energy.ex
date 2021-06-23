defmodule ViaControllers.FixedWing.Tecs.Energy do
  require ViaUtils.Constants, as: VC
  require Logger

  defstruct [
    :altitude_kp,
    :ki,
    :kd,
    :ff,
    :time_constant,
    :energy_rate_scalar,
    :output_min,
    :output_max,
    :integrator_range_min,
    :integrator_range_max,
    :pv_integrator,
    :pv_correction_prev,
    :groundspeed_prev,
    :output
  ]

  @spec new(list()) :: struct()
  def new(config) do
    %ViaControllers.FixedWing.Tecs.Energy{
      altitude_kp: Keyword.get(config, :altitude_kp, 0),
      ki: Keyword.get(config, :ki, 0),
      kd: Keyword.get(config, :kd, 0),
      ff: Keyword.get(config, :ff, nil),
      time_constant: Keyword.get(config, :time_constant, 1.0),
      energy_rate_scalar: Keyword.fetch!(config, :energy_rate_scalar),
      output_min: Keyword.fetch!(config, :output_min),
      output_max: Keyword.fetch!(config, :output_max),
      integrator_range_min: -Keyword.get(config, :integrator_range, 0),
      integrator_range_max: Keyword.get(config, :integrator_range, 0),
      pv_integrator: 0,
      pv_correction_prev: 0,
      groundspeed_prev: nil,
      output: Keyword.fetch!(config, :output_neutral)
    }
  end

  @spec update(struct(), map(), map(), number(), number()) :: tuple()
  def update(tecs, cmds, values, _airspeed, dt_s) do
    # Speeds are [m/s]
    # Altitudes are [m]
    # Logger.debug("cmds: #{ViaUtils.Format.eftb_map(cmds, 3)}")
    # Logger.debug("vals: #{ViaUtils.Format.eftb_map(values, 3)}")
    groundspeed = values.groundspeed_mps
    energy_rate_scalar = tecs.energy_rate_scalar

    groundspeed_dot =
      if is_nil(tecs.groundspeed_prev) do
        0
      else
        (groundspeed - tecs.groundspeed_prev) / dt_s
      end

    altitude_corr = cmds.altitude_corr_m
    alt_rate = altitude_corr * tecs.altitude_kp
    potential_energy_rate_sp = alt_rate * VC.gravity()

    kinetic_energy_rate = groundspeed * groundspeed_dot
    energy_rate = kinetic_energy_rate + values.potential_energy_rate

    kinetic_energy_rate_sp = cmds.kinetic_energy_rate
    energy_rate_sp = kinetic_energy_rate_sp + potential_energy_rate_sp

    energy_corr = cmds.energy - values.energy
    energy_rate_corr = energy_rate_sp - energy_rate

    # Logger.debug("e/e_sp/edot/edot_sp: #{Common.Utils.eftb(values.energy,3)}/#{Common.Utils.eftb(cmds.energy,3)}/#{Common.Utils.eftb(energy_rate,3)}/#{Common.Utils.eftb(energy_rate_sp, 3)}")

    cmd_p = energy_corr * energy_rate_scalar

    # Logger.debug("ecorr: #{Common.Utils.eftb(energy_corr,0)}")
    in_range =
      ViaUtils.Math.in_range?(
        energy_corr,
        tecs.integrator_range_min,
        tecs.integrator_range_max
      )

    pv_integrator =
      if in_range do
        pv_add = energy_corr * dt_s
        tecs.pv_integrator + pv_add * energy_rate_scalar
      else
        tecs.pv_integrator
      end

    cmd_i =
      (tecs.ki * pv_integrator)
      |> ViaUtils.Math.constrain(-0.4, 0.4)

    cmd_d = energy_rate_corr * tecs.kd * energy_rate_scalar

    delta_output = (cmd_p + cmd_i + cmd_d) / tecs.time_constant

    feed_forward =
      case Map.get(tecs, :ff) do
        nil ->
          0

        f ->
          f.(energy_rate_sp, energy_rate, cmds.groundspeed_mps)
          |> ViaUtils.Math.constrain(tecs.output_min, tecs.output_max)
      end

    output =
      (feed_forward + delta_output)
      |> ViaUtils.Math.constrain(tecs.output_min, tecs.output_max)

    # Prevent integrator wind-up
    pv_integrator =
      if tecs.ki > 0 do
        # Logger.debug("pv_int: #{pv_integrator}/ #{cmd_i/tecs.ki}")
        # Common.Utils.Math.constrain(cmd_i/tecs.ki, tecs.output_min, tecs.output_max)
        cmd_i / tecs.ki
      else
        0.0
      end

    # Logger.debug("tecs eng: #{Common.Utils.eftb(output,2)}}")
    # Logger.debug("p/i/d/ff/total: #{ViaUtils.Format.eftb(cmd_p,3)}/#{ViaUtils.Format.eftb(cmd_i,3)}/#{ViaUtils.Format.eftb(cmd_d, 3)}/#{ViaUtils.Format.eftb(feed_forward,3)}/#{ViaUtils.Format.eftb(output, 3)}")
    {%{
       tecs
       | output: output,
         groundspeed_prev: groundspeed,
         pv_correction_prev: energy_corr,
         pv_integrator: pv_integrator
     }, output}
  end
end
