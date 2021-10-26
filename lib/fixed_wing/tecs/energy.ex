defmodule ViaControllers.FixedWing.Tecs.Energy do
  require ViaUtils.Constants, as: VC
  require Logger

  defstruct [
    :altitude_kp,
    :speed_kp,
    :ki,
    :kd,
    :feed_forward_multiplier,
    :time_constant,
    :energy_rate_scalar,
    :output_min,
    :output_max,
    :integrator_range_min,
    :integrator_range_max,
    :pv_integrator,
    :pv_correction_prev,
    :groundspeed_prev_mps,
    :groundspeed_dot_prev_mpss,
    :output
  ]

  @spec new(list()) :: struct()
  def new(config) do
    feed_forward_speed_max_mps = Keyword.get(config, :feed_forward_speed_max_mps, nil)

    feed_forward_multiplier =
      if is_nil(feed_forward_speed_max_mps),
        do: 0,
        else: 1 / (feed_forward_speed_max_mps * feed_forward_speed_max_mps)

    %ViaControllers.FixedWing.Tecs.Energy{
      altitude_kp: Keyword.get(config, :altitude_kp, 0),
      speed_kp: Keyword.get(config, :speed_kp, 0),
      ki: Keyword.get(config, :ki, 0),
      kd: Keyword.get(config, :kd, 0),
      feed_forward_multiplier: feed_forward_multiplier,
      time_constant: Keyword.get(config, :time_constant, 1.0),
      energy_rate_scalar: Keyword.fetch!(config, :energy_rate_scalar),
      output_min: Keyword.fetch!(config, :output_min),
      output_max: Keyword.fetch!(config, :output_max),
      integrator_range_min: -Keyword.get(config, :integrator_range, 0),
      integrator_range_max: Keyword.get(config, :integrator_range, 0),
      pv_integrator: 0,
      pv_correction_prev: 0,
      groundspeed_prev_mps: 0,
      groundspeed_dot_prev_mpss: 0,
      output: Keyword.fetch!(config, :output_neutral)
    }
  end

  @spec update(struct(), map(), map()) :: tuple()
  def update(tecs, cmds, values) do
    %{
      altitude_kp: altitude_kp,
      speed_kp: speed_kp,
      ki: ki,
      kd: kd,
      feed_forward_multiplier: feed_forward_multiplier,
      time_constant: time_constant,
      energy_rate_scalar: energy_rate_scalar,
      output_min: output_min,
      output_max: output_max,
      integrator_range_min: integrator_range_min,
      integrator_range_max: integrator_range_max,
      pv_integrator: pv_integrator,
      pv_correction_prev: _pv_correction_prev,
      groundspeed_prev_mps: groundspeed_prev_mps,
      groundspeed_dot_prev_mpss: groundspeed_dot_prev_mpss
    } = tecs

    %{
      potential_energy: potential_energy_sp,
      kinetic_energy: kinetic_energy_sp,
      kinetic_energy_rate: kinetic_energy_rate_sp,
      altitude_corr_m: altitude_corr_m,
      groundspeed_mps: cmd_groundspeed_mps
    } = cmds

    %{
      potential_energy: potential_energy,
      kinetic_energy: kinetic_energy,
      potential_energy_rate: potential_energy_rate,
      groundspeed_mps: groundspeed_mps,
      # airspeed: airspeed_mps,
      dt_s: dt_s
    } = values

    # Logger.debug("cmds: #{ViaUtils.Format.eftb_map(cmds, 3)}")
    # Logger.debug("vals: #{ViaUtils.Format.eftb_map(values, 3)}")
    # groundspeed_prev_mps =
    #   if is_nil(groundspeed_prev_mps), do: groundspeed_mps, else: groundspeed_prev_mps

    groundspeed_dot_mpss_new = (groundspeed_mps - groundspeed_prev_mps) / dt_s
    groundspeed_dot_mpss = 0.95 * groundspeed_dot_prev_mpss + 0.05 * groundspeed_dot_mpss_new
    # if is_nil(groundspeed_prev_mps) do
    #   0
    # else
    # end

    alt_rate = altitude_corr_m
    potential_energy_rate_sp = alt_rate * VC.gravity()

    kinetic_energy_rate = groundspeed_mps * groundspeed_dot_mpss

    # Logger.debug(
    #   "dt/gs/gs_prev/gsdot/ke_rate: #{ViaUtils.Format.eftb(dt_s, 4)}/#{ViaUtils.Format.eftb(groundspeed_mps, 2)}/#{ViaUtils.Format.eftb(groundspeed_prev_mps, 2)}/#{ViaUtils.Format.eftb(groundspeed_dot_mpss, 2)}/#{ViaUtils.Format.eftb(kinetic_energy_rate, 1)}"
    # )

    # energy_rate = kinetic_energy_rate + potential_energy_rate

    # energy_rate_sp = kinetic_energy_rate_sp * speed_kp + potential_energy_rate_sp * altitude_kp

    # energy_sp = potential_energy_sp + kinetic_energy_sp

    potential_energy_corr = (potential_energy_sp - potential_energy) * altitude_kp
    kinetic_energy_corr = (kinetic_energy_sp - kinetic_energy) * speed_kp
    energy_corr = potential_energy_corr + kinetic_energy_corr

    potential_energy_rate_corr = (potential_energy_rate_sp - potential_energy_rate) * altitude_kp
    kinetic_energy_rate_corr = (kinetic_energy_rate_sp - kinetic_energy_rate) * speed_kp

    energy_rate_corr = potential_energy_rate_corr + kinetic_energy_rate_corr

    # str =
    #   "pe_corr/ke_corr/pedot_corr/kedot_corr: " <>
    #     "#{ViaUtils.Format.eftb(potential_energy_corr, 3)}/" <>
    #     "#{ViaUtils.Format.eftb(kinetic_energy_corr, 3)}/" #<>
        # "#{ViaUtils.Format.eftb(potential_energy_rate_corr, 3)}/" <>
        # "#{ViaUtils.Format.eftb(kinetic_energy_rate_corr, 3)}/"

    # Logger.debug(str)

    cmd_p = energy_corr * energy_rate_scalar

    # Logger.debug("ecorr: #{Common.Utils.eftb(energy_corr,0)}")
    in_range =
      ViaUtils.Math.in_range?(
        energy_corr,
        integrator_range_min,
        integrator_range_max
      )

    pv_integrator =
      if in_range do
        pv_add = energy_corr * dt_s
        pv_integrator + pv_add * energy_rate_scalar
      else
        pv_integrator
      end

    cmd_i =
      (ki * pv_integrator)
      |> ViaUtils.Math.constrain(-0.4, 0.4)

    cmd_d = energy_rate_corr * kd * energy_rate_scalar

    delta_output = (cmd_p + cmd_i + cmd_d) / time_constant

    feed_forward =
      (cmd_groundspeed_mps * cmd_groundspeed_mps * feed_forward_multiplier)
      |> ViaUtils.Math.constrain(output_min, output_max)

    output =
      (feed_forward + delta_output)
      |> ViaUtils.Math.constrain(output_min, output_max)

    # Prevent integrator wind-up
    pv_integrator =
      if ki > 0 do
        # Logger.debug("pv_int: #{pv_integrator}/ #{cmd_i/tecs.ki}")
        # Common.Utils.Math.constrain(cmd_i/tecs.ki, tecs.output_min, tecs.output_max)
        cmd_i / ki
      else
        0.0
      end

    # Logger.debug("tecs eng: #{Common.Utils.eftb(output,2)}}")
    # Logger.debug("p/i/d/ff/total: #{ViaUtils.Format.eftb(cmd_p,3)}/#{ViaUtils.Format.eftb(cmd_i,3)}/#{ViaUtils.Format.eftb(cmd_d, 3)}/#{ViaUtils.Format.eftb(feed_forward,3)}/#{ViaUtils.Format.eftb(output, 3)}")
    {%{
       tecs
       | output: output,
         groundspeed_prev_mps: groundspeed_mps,
         groundspeed_dot_prev_mpss: groundspeed_dot_mpss,
         pv_correction_prev: energy_corr,
         pv_integrator: pv_integrator
     }, output}
  end
end
