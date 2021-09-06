defmodule ViaControllers.MixProject do
  use Mix.Project

  @version "0.1.1"
  @source_url "https://github.com/copperpunk-elixir/via-controllers"

  def project do
    [
      app: :via_controllers,
      version: @version,
      elixir: "~> 1.11",
      description: description(),
      package: package(),
      source_url: @source_url,
      docs: docs(),
      start_permanent: Mix.env() == :prod,
      deps: deps()
    ]
  end

  # Run "mix help compile.app" to learn about applications.
  def application do
    [
      extra_applications: [:logger]
    ]
  end

  defp description do
    "Control functions for use with Via autopilot. Documentation is INCOMPLETE."
  end

  defp package do
    %{
      licenses: ["GPL-3.0"],
      links: %{"Github" => @source_url}
    }
  end

  defp docs do
    [
      extras: ["README.md"],
      main: "readme",
      source_ref: "v#{@version}",
      source_url: @source_url
    ]
  end

  # Run "mix help deps" to learn about dependencies.
  defp deps do
    [
      {:ex_doc, "~> 0.24", only: :dev, runtime: false},
      {:via_utils,git: "https://github.com/copperpunk-elixir/via-utils.git", tag: "v0.1.4-alpha"},
      # {:dep_from_hexpm, "~> 0.3.0"},
      # {:dep_from_git, git: "https://github.com/elixir-lang/my_dep.git", tag: "0.1.0"}
    ]
  end
end
