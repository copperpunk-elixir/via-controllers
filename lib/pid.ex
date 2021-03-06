defmodule ViaControllers.Pid do
  require Logger
  require ViaUtils.Shared.ControlTypes, as: SCT

  defstruct [
    :kp,
    :ki,
    :kd,
    :ff_function,
    :output_min,
    :output_neutral,
    :output_max,
    :integrator_range_min,
    :integrator_range_max,
    :integrator_airspeed_min_mps,
    :integrator,
    :correction_prev,
    :output
  ]

  @spec new(list()) :: struct()
  def new(config) do
    output_neutral = Keyword.fetch!(config, SCT.output_neutral())

    %ViaControllers.Pid{
      kp: Keyword.fetch!(config, SCT.kp()),
      ki: Keyword.fetch!(config, SCT.ki()),
      kd: Keyword.fetch!(config, SCT.kd()),
      ff_function: get_ff_function(config),
      output_min: Keyword.fetch!(config, SCT.output_min()),
      output_neutral: output_neutral,
      output_max: Keyword.fetch!(config, SCT.output_max()),
      integrator_range_min: -Keyword.fetch!(config, SCT.integrator_range()),
      integrator_range_max: Keyword.fetch!(config, SCT.integrator_range()),
      integrator_airspeed_min_mps: Keyword.fetch!(config, SCT.integrator_airspeed_min_mps()),
      integrator: 0,
      correction_prev: 0,
      output: output_neutral
    }
  end

  @spec update(struct(), number(), number(), number(), number(), boolean()) :: tuple()
  def update(pid, cmd, value, airspeed_mps, dt_s, debug \\ false) do
    %{
      integrator_range_min: integrator_range_min,
      integrator_range_max: integrator_range_max,
      integrator: integrator,
      integrator_airspeed_min_mps: integrator_airspeed_min_mps,
      kp: kp,
      ki: ki,
      kd: kd,
      correction_prev: correction_prev,
      ff_function: ff_function,
      output_min: output_min,
      output_neutral: output_neutral,
      output_max: output_max
    } = pid

    correction = cmd - value

    in_range =
      ViaUtils.Math.in_range?(
        correction,
        integrator_range_min,
        integrator_range_max
      )

    # if debug do
    #   Logger.debug(
    #     "corr/int_min/int_max/in_range:  #{ViaUtils.Format.eftb(correction, 3)}/#{ViaUtils.Format.eftb(integrator_range_min, 3)}#{ViaUtils.Format.eftb(integrator_range_max, 3)}/#{in_range}"
    #   )
    # end

    value_add_to_integrator = if in_range, do: correction * dt_s, else: 0

    integrator =
      if airspeed_mps > integrator_airspeed_min_mps,
        do: integrator + value_add_to_integrator,
        else: 0

    cmd_p = kp * correction
    cmd_i = ki * integrator
    cmd_d = if dt_s != 0, do: -kd * (correction - correction_prev) / dt_s, else: 0

    # TODO: Incorporate airspeed into this calculate. Use nth-order equation to drop off FF from
    # max_value at airspeed=0, to min_value at airspeed=airspeed_max
    # For now we are ignoring airspeed
    feed_forward = ff_function.(value + correction, airspeed_mps)

    output =
      (cmd_p + cmd_i + cmd_d + feed_forward + output_neutral)
      |> ViaUtils.Math.constrain(output_min, output_max)

    if debug do
      Logger.debug(
        "cmd/value/corr/p/i/d/ff/out: #{ViaUtils.Format.eftb(cmd, 3)}/#{ViaUtils.Format.eftb(value, 3)}/#{ViaUtils.Format.eftb(correction, 3)}/#{ViaUtils.Format.eftb(cmd_p, 3)}/#{ViaUtils.Format.eftb(cmd_i, 3)}/#{ViaUtils.Format.eftb(cmd_d, 3)}/#{ViaUtils.Format.eftb(feed_forward, 3)}/#{ViaUtils.Format.eftb(output, 3)}"
      )
    end

    integrator = if ki != 0, do: cmd_i / ki, else: 0

    {%{pid | output: output, correction_prev: correction, integrator: integrator}, output}
  end

  @spec output(struct()) :: number()
  def output(pid) do
    pid.output
  end

  @spec reset_integrator(struct()) :: struct()
  def reset_integrator(pid) do
    %{pid | integrator: 0}
  end

  @spec get_ff_function(list()) :: function()
  def get_ff_function(config) do
    with function <- Keyword.get(config, SCT.feed_forward_function()) do
      if is_nil(function) do
        multiplier = Keyword.fetch!(config, SCT.feed_forward_multiplier())
        fn cmd, _airspeed_mps -> cmd * multiplier end
      else
        function
      end
    end
  end
end
