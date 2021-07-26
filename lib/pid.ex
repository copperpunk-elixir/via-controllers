defmodule ViaControllers.Pid do
  defstruct [
    :kp,
    :ki,
    :kd,
    :ff_multiplier,
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
    output_neutral = Keyword.fetch!(config, :output_neutral)

    %ViaControllers.Pid{
      kp: Keyword.fetch!(config, :kp),
      ki: Keyword.fetch!(config, :ki),
      kd: Keyword.fetch!(config, :kd),
      ff_multiplier: Keyword.fetch!(config, :ff_multiplier),
      output_min: Keyword.fetch!(config, :output_min),
      output_neutral: output_neutral,
      output_max: Keyword.fetch!(config, :output_max),
      integrator_range_min: -Keyword.fetch!(config, :integrator_range),
      integrator_range_max: Keyword.fetch!(config, :integrator_range),
      integrator_airspeed_min_mps: Keyword.fetch!(config, :integrator_airspeed_min_mps),
      integrator: 0,
      correction_prev: 0,
      output: output_neutral
    }
  end

  @spec update(struct(), number(), number(), number(), number()) :: tuple()
  def update(pid, cmd, value, airspeed_mps, dt_s) do
    correction = cmd - value

    in_range =
      ViaUtils.Math.in_range?(
        correction,
        pid.integrator_range_min,
        pid.integrator_range_max
      )

    value_add_to_integrator = if in_range, do: correction * dt_s, else: 0

    integrator =
      if airspeed_mps > pid.integrator_airspeed_min_mps,
        do: pid.integrator + value_add_to_integrator,
        else: 0

    cmd_p = pid.kp * correction
    cmd_i = pid.ki * integrator
    cmd_d = if dt_s != 0, do: -pid.kd * (correction - pid.correction_prev) / dt_s, else: 0

    # TODO: Incorporate airspeed into this calculate. Use nth-order equation to drop off FF from
    # max_value at airspeed=0, to min_value at airspeed=airspeed_max
    # For now we are ignoring airspeed
    feed_forward = (value + correction) * pid.ff_multiplier

    output =
      (cmd_p + cmd_i + cmd_d + feed_forward + pid.output_neutral)
      |> ViaUtils.Math.constrain(pid.output_min, pid.output_max)

    # if pid.process_variable == :course_rotate do# and pid.control_variable == :thrust do
    #   Logger.debug("cmd/value/corr/p/i/d/ff/dO/out: #{Common.Utils.eftb(cmd,3)}/#{Common.Utils.eftb(value,3)}/#{Common.Utils.eftb(correction,3)}/#{Common.Utils.eftb(cmd_p, 3)}/#{Common.Utils.eftb(cmd_i, 3)}/#{Common.Utils.eftb(cmd_d, 3)}/#{Common.Utils.eftb(feed_forward,3)}/#{Common.Utils.eftb(delta_output, 3)}/#{Common.Utils.eftb(output, 3)}")
    # end

    integrator = if pid.ki != 0, do: cmd_i / pid.ki, else: 0

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
end
