# Measure black pins on white background with OpenCV

## Description
This program measures height and width of multiple pins (black pin on white background) using OpenCV. The program also calulates slope of the base and measures height of the pin from this base. Base must be on the bottom of the image.

## Done
- detect edges of the image
- detect if base is not parallel to the x axis and rotate picture so it is parallel,
- create height map (easy to calculate max height, but we want tu detect if there are more than 1 pin to measure

## ToDo
- detect if there is more than 1 object to measure and make segmentation (derivative based method that uses height map?),
- make measures of the height (height map done - easy to measure max height, harder to detect multiple objects to measure),
- make measures of the width (using line iterator),

## Requirements
- OpenCV 3.x or 4.x

## Screens
<p align="center">
  <img src="https://raw.githubusercontent.com/Kuwashitamidayo/measure_object/master/screens/screen.png">
</p>
