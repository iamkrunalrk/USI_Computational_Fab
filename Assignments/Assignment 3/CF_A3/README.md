# Halftoning

This code implements 3 algorithms for converting grayscale images to black and white images. These techniques are  used to create the illusion of grayscale tones using only black and white pixels.

The code includes functions for:

* Reading a grayscale image from a PGM file.
* Writing a black and white image to a PGM file.
* Converting a grayscale image to black and white using thresholding.
* Converting a grayscale image to black and white using dithering.
* Converting a grayscale image to black and white using error diffusion.

## Compiling instructions

The code requires a C++ compiler. You can compile the code using a command like this:

Open your terminal, go to the directory where the C++ file stays: 
```bash
cd <WORKING DIR>
```
 For example: 
```bash
cd /Users/usr/Downloads/CF_A3/
```


run: 
```bash
g++ -o Halftoning halftoning.cpp
``` 
This will create an executable file named Halftoning.

## Running the executable file

The code takes three arguments:

1. Input Image File Name: The name of the grayscale image file in PGM format.
2. Output Image File Name: The name of the output black and white image file in PGM format.
3. Method: The method used for conversion. Valid options are "Thresholding", "Dithering", or "ErrorDiffusion".

```bash
./Halftoning <Input_Image> <Outout_Image> <Method>
``` 

Here is an example of how to run the code to convert an image named "test1.pgm" to a black and white image named "test_1_output_Dithering.pgm" using dithering:: 
```bash
./Halftoning test1.pgm test_1_output_Dithering.pgm Dithering
```