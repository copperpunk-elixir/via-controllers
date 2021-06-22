defmodule Scalar.RateControlTest do
  use ExUnit.Case
  require Logger

  setup do
    {:ok, []}
  end

  test "Rate Control Test" do
    output_min = -100
    output_max = 100
    output_neutral = 0
    multiplier = 5
    rate_max = 3

    s =
      Scalar.new(output_min, output_neutral, output_max, multiplier, rate_max)
      |> Scalar.update(0, 0, 1)

    dt = 0.1
    current_value = 0
    command = 10

    assert Scalar.output(s) != (command - current_value) * multiplier

    s =
    Enum.reduce(0..35, s, fn _x,acc ->
    Scalar.update(acc, command, current_value, dt)
    end)

    assert Scalar.output(s) == (command - current_value) * multiplier
  end
end
