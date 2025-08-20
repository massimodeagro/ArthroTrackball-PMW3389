# ArthroTrackball-PMW3389
3D printing, circuit design and code for the sensor-based ArtroTrackball system: an omnidirectional treadmill desgned for jumping spiders.

The system has been designed thanks to a [research grant from the Association for the Study of Animal behaviour (ASAB)](https://www.asab.org/research-grants) and to a [post-doc fellowship from the Accademia Nazionale dei Lincei](https://www.lincei.it/it/premi_borse/borsa-di-studio-post-dottorato-dott-giuseppe-guelfi-2025)

The system uses two [PMW3389 laser sensors](https://www.pixart.com/products-detail/4/PMW3389DM-T3QU) arranged in a non-coplanar fashon to calculate the rotation of the 38mm sphere around its center across the x, y and z axes. The SROM for the sensor is provided by Pixart. The code implemented to do so has been written following the algorithms described in [M. Kumagai and R. L. Hollis, "Development of a three-dimensional ball rotation sensing system using optical mouse sensors," 2011 IEEE International Conference on Robotics and Automation, Shanghai, China, 2011, pp. 5038-5043](https://ieeexplore.ieee.org/abstract/document/5979899).

Here two code are provided. One to be flashed on an Arduino Uno device, and the second to be written on a Raspberry Pi Pico. Code to interface with PMW3389 device has been modified from https://github.com/mrjohnk/PMW3389DM. The sensor also requires a breakout board. The sensor with the board can be bought from https://www.tindie.com/products/citizenjoe/pmw3389-motion-sensor/. The former code achieves speed of up to 400Hz, while the Pi only gets to 200Hz. Both solutions then provide the x, y and z rotations as comma separated signed float (positive for clockwise, negative for counterclockwise). Any serial reader can then be used to retrive the values. I have also written a python package (AtrhroTrackball) that implements a class which can read directly the system output and sincronize it with other systems (camera, monitor, etc.) The package will be released soon.

If you use this, please reference at least one of the papers where I have described the system (below a not-necessarily-updated list):

- [Beydizada, N., Cannone, F., Pekàr, S., Baracchi, D., De Agrò, M. Habituation to visual stimuli is independent of boldness in a jumping spider (2024) Animal Behaviour](https://doi.org/10.1016/j.anbehav.2024.04.010)
- [Loconsole, M., Ferrante, F., Giacomazzi, D., De Agrò, M. Independence and synergy of spatial attention in the two visual systems of jumping spiders (2024) Journal of Experimental Biology](https://doi.org/10.1242/jeb.246199)
- [De Agrò, M.,  Rößler D.C., Shamble, P.S. Eye-specific detection and a multi-eye integration model of biological motion perception (2024) Journal of Experimental Biology](https://doi.org/10.1242/jeb.247061)
- [De Agrò, M., Rößler, D. C., Kim, K., & Shamble, P. S. Perception of biological motion in point-light displays by jumping spiders. (2021) PLOS Biology](https://doi.org/10.1371/journal.pbio.3001172)
