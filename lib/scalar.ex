defmodule Scalar do
  require Logger
  defstruct output_min: 0,
            output_max: 0,
            output_neutral: 0,
            multiplier: 0,
            command_rate_max: 0,
            output: 0,
            current_command: 0


  @spec new(list()) :: struct()
  def new(config) do
    initial_command = Keyword.get(config, :initial_command, nil)
    new(config[:output_min], config[:output_neutral], config[:output_max], config[:multiplier], config[:command_rate_max], initial_command)
  end

  @spec new(number(), number(), number(), number(), number(), any()) :: struct()
  def new(output_min, output_neutral, output_max, multiplier, command_rate_max, initial_command \\ nil) do
    %Scalar{
      output_min: output_min,
      output_neutral: output_neutral,
      output_max: output_max,
      multiplier: multiplier,
      command_rate_max: command_rate_max,
      output: initial_command,
      current_command: initial_command
    }
  end

  @spec update(struct(), number(), number(), number()) :: tuple()
  def update(scalar, command, current_value, dt) do
    scalar =
      if is_nil(scalar.current_command), do: %{scalar | current_command: command}, else: scalar

    command_min = scalar.current_command - scalar.command_rate_max * dt
    command_max = scalar.current_command + scalar.command_rate_max * dt

    command_rate_controlled = ViaUtils.Math.constrain(command, command_min, command_max)

    # Logger.debug("command min/max/rc: #{command_min}/#{command_max}/#{command_rate_controlled}")

    output =
      (scalar.multiplier * (command_rate_controlled - current_value) + scalar.output_neutral)
      |> ViaUtils.Math.constrain(scalar.output_min, scalar.output_max)

    {%{scalar | output: output, current_command: command_rate_controlled}, output}
  end

  @spec output(struct()) :: number()
  def output(scalar) do
    scalar.output
  end
end
