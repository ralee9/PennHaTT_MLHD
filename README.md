# PennHaTT_MLHD
Public release of the Penn Haptic Texture Toolkit, adapted for use on the Magnetically Levitated Haptic Device (MLHD).

The Penn Haptic Texture Toolkit (http://repository.upenn.edu/meam_papers/299/) is a collection of 100 haptic texture
and friction models, originally written to be rendered on a SenSable Phantom Omni. The toolkit was first developed 
by Heather Culbertson, Juan Jose Lopez Delgado, and Katherine Kuchenbecker at the University of Pennsylvania.

In this implementation, the toolkit was adapted for use with the Magnetically Levitated Haptic Device, developed by 
Professor Ralph Hollis (Robotics Institute, Carnegie Mellon University) and currently commercialized 
by Butterfly Haptics (Pittsburgh, PA).

Detailed instructions to build the project can be found in NOTES.txt. For comparison, the original binary for the
Haptic Texture Toolkit can be found in /build/TexturedSphere. The original source code, texture models, and recorded
data for the Haptic Texture Toolkit may be found at the Penn repository linked above.

CMake 2.6.0, OpenGL, and GLUT are required to build the project. MLHD libraries and headers are included.
