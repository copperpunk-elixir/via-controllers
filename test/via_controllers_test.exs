defmodule ViaControllersTest do
  use ExUnit.Case
  doctest ViaControllers

  test "greets the world" do
    assert ViaControllers.hello() == :world
  end
end
