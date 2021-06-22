defmodule ViaControllers do
  @moduledoc """
  Documentation for `ViaControllers`.
  """

  @doc """
  Hello world.

  ## Examples

      iex> ViaControllers.hello()
      :world

  """
  def hello do
    :world
  end

  @spec constrain(number(), number(), number()) :: number()
  def constrain(x, min_value, max_value) do
    case x do
      _ when x > max_value -> max_value
      _ when x < min_value -> min_value
      x -> x
    end
  end
end
