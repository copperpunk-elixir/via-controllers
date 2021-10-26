defmodule ViaControllers.FixedWing.Tecs.Balance do
  require ViaUtils.Constants, as: VC
  require Logger

  defstruct [
    :altitude_kp,
    :ki,
    :kd,
    :time_constant,
    :balance_rate_scalar,
    :min_airspeed_for_climb_mps,
    :output_min,
    :output_max,
    :integrator_range_min,
    :integrator_range_max,
    :pv_integrator,
    :integrator_factor,
    :pv_correction_prev,
    :output
  ]

  @spec new(list()) :: struct()
  def new(config) do
    %ViaControllers.FixedWing.Tecs.Balance{
      altitude_kp: Keyword.get(config, :altitude_kp, 0),
      ki: Keyword.get(config, :ki, 0),
      kd: Keyword.get(config, :kd, 0),
      time_constant: Keyword.get(config, :time_constant, 1.0),
      balance_rate_scalar: Keyword.fetch!(config, :balance_rate_scalar),
      min_airspeed_for_climb_mps: Keyword.fetch!(config, :min_airspeed_for_climb_mps),
      output_min: Keyword.fetch!(config, :output_min),
      output_max: Keyword.fetch!(config, :output_max),
      integrator_range_min: -Keyword.get(config, :integrator_range, 0),
      integrator_range_max: Keyword.get(config, :integrator_range, 0),
      pv_integrator: 0,
      integrator_factor: Keyword.get(config, :integrator_factor, 1),
      pv_correction_prev: 0,
      output: Keyword.fetch!(config, :output_neutral)
    }
  end

  @spec update(struct(), map(), map()) :: tuple()
  def update(tecs, cmds, values) do
    %{
      altitude_kp: altitude_kp,
      ki: ki,
      kd: kd,
      time_constant: time_constant,
      balance_rate_scalar: balance_rate_scalar,
      min_airspeed_for_climb_mps: min_airspeed_for_climb_mps,
      output_min: output_min,
      output_max: output_max,
      integrator_range_min: integrator_range_min,
      integrator_range_max: integrator_range_max,
      pv_integrator: pv_integrator,
      integrator_factor: integrator_factor
    } = tecs

    %{
      # kinetic_energy: kinetic_energy_sp,
      # kinetic_energy_rate: kinetic_energy_rate_sp,
      altitude_corr: altitude_corr
      # groundspeed_mps: cmd_groundspeed_mps
    } = cmds

    %{
      # kinetic_energy: kinetic_energy,
      potential_energy: potential_energy,
      potential_energy_rate: potential_energy_rate,
      airspeed_mps: airspeed_mps,
      # groundspeed_mps: groundspeed_mps,
      dt_s: dt_s
    } = values

    excess_airspeed_mps =
      ViaUtils.Math.constrain(airspeed_mps - min_airspeed_for_climb_mps, -5, 5)

    altitude_corr_mult = 0.5 * (:math.sin(excess_airspeed_mps * :math.pi() / 10) + 1)

    # Logger.debug(
    #   "exs_ap/mult/pre: #{ViaUtils.Format.eftb(excess_airspeed_mps, 3)}/#{ViaUtils.Format.eftb(altitude_corr_mult, 3)}/#{ViaUtils.Format.eftb(altitude_corr_mult * altitude_corr * altitude_kp, 1)}"
    # )

    alt_rate_sp =
      (altitude_corr_mult * altitude_corr * altitude_kp) |> ViaUtils.Math.constrain(-5.0, 5.0)

    # if airspeed_mps > min_airspeed_for_climb_mps do
    #   (altitude_corr * altitude_kp) |> ViaUtils.Math.constrain(-5.0, 5.0)
    # else
    #   (altitude_corr * altitude_kp) |> ViaUtils.Math.constrain(-0.5, 0.5)
    # end

    # Logger.debug(
    #   "alt_corr/alt_rate_sp: #{ViaUtils.Format.eftb(altitude_corr, 1)}/#{ViaUtils.Format.eftb(alt_rate_sp, 2)}"
    # )

    potential_energy_rate_sp = alt_rate_sp * VC.gravity()
    potential_energy_sp = potential_energy + potential_energy_rate_sp * dt_s

    balance_cmd = potential_energy_sp
    balance_value = potential_energy
    balance_corr = balance_cmd - balance_value
    balance_rate_cmd = potential_energy_rate_sp
    balance_rate_value = potential_energy_rate
    balance_rate_corr = balance_rate_cmd - balance_rate_value

    # Logger.debug(
    #   "bratecmd/brateval: #{ViaUtils.Format.eftb(balance_rate_cmd, 3)}/#{ViaUtils.Format.eftb(balance_rate_value, 3)}"
    # )

    # Logger.debug(
    #   "bcorr/brate_corr: #{ViaUtils.Format.eftb(balance_corr, 3)}/#{ViaUtils.Format.eftb(balance_rate_corr, 3)}/#{ViaUtils.Format.eftb(integrator_range_max, 3)}"
    # )

    # Logger.debug(
    #   "pe_sp/pe: #{ViaUtils.Format.eftb(potential_energy_sp, 1)}/#{ViaUtils.Format.eftb(potential_energy, 1)}"
    # )

    # Logger.debug(
    #   "rate: pe_sp/pe: #{ViaUtils.Format.eftb(potential_energy_rate_sp, 3)}/#{ViaUtils.Format.eftb(potential_energy_rate, 3)}"
    # )

    # Proportional
    cmd_p = balance_corr
    # Integrator
    # Logger.debug(
    #   "bcorr/pv_int: #{ViaUtils.Format.eftb(balance_corr, 3)}/#{ViaUtils.Format.eftb(tecs.integrator_range_max, 3)}"
    # )

    in_range = ViaUtils.Math.in_range?(balance_corr, integrator_range_min, integrator_range_max)

    error_positive = cmd_p > 0
    i_positive = pv_integrator > 0

    pv_mult = if !i_positive and !error_positive, do: 1.0, else: integrator_factor

    pv_add = balance_corr * dt_s

    pv_integrator =
      cond do
        in_range -> pv_integrator + pv_add * pv_mult
        error_positive != i_positive -> pv_integrator + pv_add * pv_mult
        true -> pv_integrator
      end

    # Logger.debug("pv int: #{ViaUtils.Format.Format.eftb(pv_integrator,3)}")

    cmd_i =
      (ki * pv_integrator)
      |> ViaUtils.Math.constrain(-1.0, 1.0)

    # Logger.info("cmd i pre/post: #{ViaUtils.Format.Format.eftb(cmd_i_mult*pv_integrator,3)}/#{ViaUtils.Format.eftb(cmd_i, 3)}")
    pv_integrator = if ki != 0, do: cmd_i / ki, else: 0
    # Derivative
    cmd_d = balance_rate_corr * kd

    cmd_rate = 0 * balance_rate_cmd * time_constant

    output =
      ((cmd_p + cmd_i + cmd_d + cmd_rate) / time_constant * balance_rate_scalar)
      |> ViaUtils.Math.constrain(output_min, output_max)

    # Logger.debug("tecs bal err/out: #{ViaUtils.Format.eftb(altitude_corr,2)}/#{ViaUtils.Format.eftb_deg(output,1)}")
    # Logger.debug(
    #   "p/i/d/total: #{ViaUtils.Format.eftb(cmd_p, 3)}/#{ViaUtils.Format.eftb(cmd_i, 3)}/#{ViaUtils.Format.eftb(cmd_d, 3)}/#{ViaUtils.Format.eftb(output, 3)}/#{ViaUtils.Format.eftb_deg(output, 1)}"
    # )

    # Logger.debug("p/i/total: #{ViaUtils.Format.eftb_deg(cmd_p,3)}/#{ViaUtils.Format.eftb_deg(cmd_i,3)}/#{ViaUtils.Format.eftb_deg(output, 3)}")

    {%{tecs | pv_integrator: pv_integrator, output: output}, output}
  end
end
