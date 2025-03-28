# Antenna Tracker

A new antenna tracker is being built that is larger. This code will be on it.

### Hardware:

- 2 Nema 22 Stepper Motors
- Azimuth gear ratio: ??? ("Around 150-1")
- Elevation gear ratio: ???
- Microcontroller: ??? (Esp32, RaspberryPi, no one knows)
- GPS (on both tracker and balloon)
- Antenna Capable of LoRa
- Turning Ranges: ???

### Explanation of Azimuth:

Azimuth is the angle relative to North (0 degrees).

![Azimuth](https://www.pveducation.org/sites/default/files/PVCDROM/Properties-of-Sunlight/Images/AZIMUTH.gif)

### Method for Finding Angles:

We are given the longitude $(\lambda)$, latitude $(\phi)$, and altitude $(h)$ of both the antenna tracker and the balloon.<br>
$(\lambda_a, \phi_a, h_a), (\lambda_b, \phi_b, h_b)$
We also know Earth's radius $r = 6378137$ and Earth's eccentricity $e = 0.08181919$  
*Note: all calculations use units of radians and meters*

Using the Earth-Centered, Earth-Fixed (ECEF) coordinate system we find:
$$ N = \frac{r}{\sqrt{1 - (e \sin(\phi))^2}} $$  
$$ X = (N + h) \cos(\phi) \cos(\lambda) $$  
$$ Y = (N + h) \cos(\phi) \sin(\lambda) $$  
$$ Z = (N \cdot (1 - e^2) + h) \sin(\phi) $$  

The relative position is then:  
$$ 
\vec{r} = \begin{bmatrix}
(X_b - X_a) \\
(Y_b - Y_a) \\
(Z_b - Z_a) \\
\end{bmatrix}
$$

Now we can use a transformation matrix to convert to the ENU (East, North, Up) frame:

$$
\begin{bmatrix}
E \\
N \\
U
\end{bmatrix}
=
\begin{bmatrix}
-\sin(\lambda_a) & \cos(\lambda_a) & 0 \\
-\sin(\phi_a) \cos(\lambda_a) & -\sin(\phi_a) \sin(\lambda_a) & \cos(\phi_a) \\
\cos(\phi_a) \cos(\lambda_a) & \cos(\phi_a) \sin(\lambda_a) & \sin(\phi_a)
\end{bmatrix}
\vec{r}
$$

Using a line-of-sight geometric check:
$$ \text{geometricHorizonAngle} = \arctan\left(\frac{U}{\sqrt{E^2 + N^2}}\right) $$  
$$ \text{correction} = \arccos\left(\frac{r}{r + h_a}\right) $$  
$$ \text{elevation} = \text{geometricHorizonAngle} - \text{correction} $$  
$$ \text{azimuth} = \arctan\left(\frac{E}{N}\right) $$  
