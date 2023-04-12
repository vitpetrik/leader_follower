using Plots

RADIUS = 5.0

function force_field(x)
    y = 0
    if x >= RADIUS
        y = 5*(x-RADIUS)^2
    else
        y = 50*(-1/(x/5)^2 + 1)
    end

    return y
end

function main()
   x = LinRange(3, 10, 5000)
   y = force_field.(x)
   plot(x,y)
end

main()

