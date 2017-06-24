
Car model: 1 box, 4 wheels, 2 directional in the front, 2 fixed in the rear

https://user-images.githubusercontent.com/4974818/27407504-d3114bda-56d8-11e7-811b-713aee5f4412.png

                            |--------------------|
                            | __           __    |
                            | \ \          \ \   |
                            |  \ \     x    \ \  |
                            |   \_\    |     \_\ |
                            |          |         |
                            |          |         |
                            |          |         |
                            |          |L        |
                            |          |         |
                            |   _      |     _   |
                            |  | |     |    | |  |
                            |  | |     X    | |  |
                            |  |_|          |_|  |
                            |                    |
                            |--------------------|

Simplified by approximation : consider that the left and right directional wheels are parallel when turning (they are not in real life; see "ackerman drive")
This simplification is the bycicle model.

https://user-images.githubusercontent.com/4974818/27407537-f78ee828-56d8-11e7-8210-84575f5f3034.png

                                |-------|
                                | __    |
                                | \ \   |
                                |  \ \  |  -|
                                |   \_\ |   |
                                |       |   |
                                |       |   |
                                |       |   |
                                |       |   | length L
                                |       |   |
                                |   _   |   |
                                |  | |  |   |
                                |  |x|  |  -|
                                |  |_|  |
                                |       |
                                |-------|

Given:
x & y       : Position point of center of the rear axle
L			: Length between front wheel and rear wheel axles
d           : Distance of movement of the vehicle
α (alpha)   : Angle of front wheel **relative** to center line of the vehicle
θ (theta)   : Angle of center line of the vehicle, **absolute**

We find:
β (beta)	: Turning angle
R           : Radius of turning circle
Cx & Cy     : Center point of turning circle
x' & y'     : New position point of center of the rear axle
θ'          : New angle of center line of the vehicle, **absolute**

β = (d / L) * tan(α)
R = d / β

If β < 0.001 : linear movement approximation

    x' = x + d * cos(θ)
    y' = y + d * sin(θ)
    θ' = (θ + β) mod 2π

Else :

    Cx = x - sin(θ) * R
    Cy = y + cos(θ) * R
    x' = Cx + sin(θ + β) * R
    y' = Cy - cos(θ + β) * R
    θ' = (θ + β) mod 2π
