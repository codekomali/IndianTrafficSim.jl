using IndianTrafficSim
using Documenter

DocMeta.setdocmeta!(IndianTrafficSim, :DocTestSetup, :(using IndianTrafficSim); recursive=true)

makedocs(;
    modules=[IndianTrafficSim],
    authors="Code Komali <code.komali@gmail.com> and contributors",
    repo="https://github.com/codekomali/IndianTrafficSim.jl/blob/{commit}{path}#{line}",
    sitename="IndianTrafficSim.jl",
    format=Documenter.HTML(;
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://codekomali.github.io/IndianTrafficSim.jl",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
    ],
)

deploydocs(;
    repo="github.com/codekomali/IndianTrafficSim.jl",
    devbranch="master",
)
