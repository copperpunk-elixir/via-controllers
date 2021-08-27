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

  @spec update(struct(), map(), map(), number(), number()) :: tuple()
  def update(tecs, cmds, values, _airspeed, dt_s) do
    altitude_corr = cmds.altitude_corr
    alt_rate_sp = altitude_corr*tecs.altitude_kp
    # Logger.debug("alt_rate_sp: #{ViaUtils.Format.eftb(alt_rate_sp,2)}")
    potential_energy = values.potential_energy
    potential_energy_rate = values.potential_energy_rate

    potential_energy_rate_sp = alt_rate_sp*VC.gravity
    potential_energy_sp = values.potential_energy + potential_energy_rate_sp*dt_s

    balance_cmd = potential_energy_sp
    balance_values = potential_energy
    balance_corr = balance_cmd - balance_values
    balance_rate_cmd = potential_energy_rate_sp
    balance_rate_values = potential_energy_rate
    balance_rate_corr = balance_rate_cmd - balance_rate_values
    # Logger.debug("pe_sp/pe: #{ViaUtils.Format.eftb(potential_energy_sp,1)}/#{ViaUtils.Format.eftb(potential_energy,1)}")
    # Logger.debug("rate: pe_sp/pe: #{ViaUtils.Format.eftb(potential_energy_rate_sp,3)}/#{ViaUtils.Format.eftb(potential_energy_rate,3)}")

    # Proportional
    cmd_p = balance_corr
    # Integrator
    # Logger.debug("bcorr/pv_int: #{ViaUtils.Format.eftb(balance_corr,3)}/#{ViaUtils.Format.eftb(tecs.integrator_range_max,3)}")
    in_range = ViaUtils.Math.in_range?(balance_corr, tecs.integrator_range_min, tecs.integrator_range_max)

    error_positive = cmd_p > 0
    i_positive = tecs.pv_integrator > 0

    pv_mult = if !i_positive and !error_positive, do: 1.0, else: tecs.integrator_factor

    pv_add = balance_corr*dt_s
    pv_integrator = cond do
      in_range -> tecs.pv_integrator + pv_add*pv_mult
      error_positive != i_positive -> tecs.pv_integrator + pv_add*pv_mult
      true -> tecs.pv_integrator
    end
    # Logger.debug("pv int: #{ViaUtils.Format.Format.eftb(pv_integrator,3)}")

    cmd_i = tecs.ki*pv_integrator
    |> ViaUtils.Math.constrain(-0.175, 0.175)
    # Logger.info("cmd i pre/post: #{ViaUtils.Format.Format.eftb(cmd_i_mult*pv_integrator,3)}/#{ViaUtils.Format.eftb(cmd_i, 3)}")
    pv_integrator =
    if (tecs.ki != 0), do: cmd_i / tecs.ki, else: 0
    # Derivative
    cmd_d = balance_rate_corr*tecs.kd

    cmd_rate = 0*balance_rate_cmd*tecs.time_constant

    output = (cmd_p + cmd_i + cmd_d + cmd_rate) / tecs.time_constant * tecs.balance_rate_scalar
    |> ViaUtils.Math.constrain(tecs.output_min, tecs.output_max)

    # Logger.debug("tecs bal err/out: #{ViaUtils.Format.eftb(altitude_corr,2)}/#{ViaUtils.Format.eftb_deg(output,1)}")
    # Logger.debug("p/i/rate/total: #{ViaUtils.Format.eftb(cmd_p,3)}/#{ViaUtils.Format.eftb(cmd_i,3)}/#{ViaUtils.Format.eftb(cmd_d, 3)}/#{ViaUtils.Format.eftb(cmd_rate,3)}/#{ViaUtils.Format.eftb_deg(output, 3)}")
    # Logger.debug("p/i/total: #{ViaUtils.Format.eftb_deg(cmd_p,3)}/#{ViaUtils.Format.eftb_deg(cmd_i,3)}/#{ViaUtils.Format.eftb_deg(output, 3)}")

    {%{tecs | pv_integrator: pv_integrator, output: output}, output}
  end
end
