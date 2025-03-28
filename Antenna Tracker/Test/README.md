

# Antenna Tracker


A new antenna tracker is being built that is larger. This code will be on it.

##### Hardware:

- 2 Nema 22 Stepper Motors
- Azimuth gear ratio: ??? ("Around 150-1")
- Elevation gear ratio: ???
- Microcontroller: ??? (Esp32, RaspberryPi, no one knows)
- GPS (on both tracker and balloon)
- Antenna Capable of LoRa
- Turning Ranges: ???

##### Explanation of Azimuth:

Azimuth is the angle relative to North (0 degrees).

![Azimuth](https://www.pveducation.org/sites/default/files/PVCDROM/Properties-of-Sunlight/Images/AZIMUTH.gif)

##### Method for Finding Angles:

We are given the longitude(λ), latitude(φ), and altitude(h) of both the antenna tracker and the balloon.
$$ (λ_a, φ_a, h_a), (λ_b, φ_b, h_b) $$
We also know Earth's radius $(r = 6378137)$ and Earth's eccentricity $(e = 0.08181919)$
*Note: all calcualations use units of radians and meters*

Using the Earth-Centered, Earth-Fixed (ECEF) coordinate system we find:
$$ N = {r \over \sqrt{1 - (e*sin(φ))^2}} $$
$$ X = {(N + h) * cos(φ) * cos(λ)} $$
$$ Y = {(N + h) * cos(φ) * sin(λ)} $$
$$ Z = {(N * (1 - e^2 + h)) * sin(φ)} $$

The relative position is then:
\[
\vec{r} 
= 
\begin{bmatrix}
(X_b - X_a) \\
(Y_b - Y_a) \\
(Z_b - Z_a) \\
\end{bmatrix}
\]

Now we can use a transformation matrix to convert to the ENU (East, North, Up) frame:

\[
\begin{bmatrix}
E \\
N \\
U \\
\end{bmatrix}
=
\begin{bmatrix}
-sin(λ_a) & cos(λ_a) & 0 \\
-sin(φ_a) * cos(λ_a) & -sin(φ_a) * sin(λ_a) & cos(φ_a) \\
cos(φ_a) * cos(λ_a) & cos(φ_a) * sin(λ_a) & sin(φ_a) \\
\end{bmatrix}
\vec{r}
\]

Using a line-of-site geometric check
$$ geometricHorizonAngle = arctan(U \over \sqrt{E^2 + N^2}) $$
$$ correction = arccos(r \over (r + h_a)) $$
$$ elevation = geometricHorizonAngle - correction $$
$$ azimuth = arctan(E \over N) $$


