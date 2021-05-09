# Measure black pins on white background with OpenCV

## Description
This program measures height and width of multiple pins (black pin on white background) using OpenCV. The program also calulates slope of the base and measures height of the pin from this base. Base must be on the bottom of the image.

## Assumptions
- image has 2 colors to make edge detection easy,
- base from whitch pins are measured in on the bottom,
- pins go from down upwards,
- pins are possible to be segmented by lines perpendicular to the base.

## Done
- detect edges of the image
- detect if base is not parallel to the x axis and rotate picture so it is parallel,
- create height map (now it is easy to calculate max height, but we want tu detect if there are more than 1 pin to measure),
- make measures of the height of each pin, store its height and paint a circle at the top of the pin.

## To do
- detect if there is more than 1 pin to measure width and make segmentation (to be checked on vector of heights),
- make measures of the width of each pin (using line iterator).

## Requirements
- OpenCV 3.x or 4.x
- g++

## Screens
<p align="center">
  <img src="https://raw.githubusercontent.com/Kuwashitamidayo/measure_object/master/screens/screen.png">
</p>
