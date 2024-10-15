#include <iostream>
#include <fstream>
#include <string>
#include <vector>


using namespace std;

vector< vector<float> > inputImage;
vector< vector<bool> > outputImage;
int height;
int width;

void read_image(char *filename){
    
    string type;
    int maxValue;
    int pixelValue;
    
    ifstream inputFile(filename);
    
    if(!inputFile.is_open()){
        cout<<"Unable to open InputImageFile"<<endl;
        return;
    }
    
    inputFile>>type>>width>>height>>maxValue;

    
    inputImage.resize(height);
    outputImage.resize(height);
    
    for (int i = 0; i < width; i++){
        inputImage[i].resize(width);
        outputImage[i].resize(width);
        for (int j = 0; j < height; j++){
            inputFile>>pixelValue;
            inputImage[i][j] = (float)pixelValue / maxValue;
        }
    }
    
}


void write_image(char *filename){
    
    ofstream outputFile(filename);
    
    if(!outputFile.is_open()){
        cout<<"Unable to open OutputImageFile"<<endl;
        return;
    }
    
    outputFile<<"P1"<<endl<<width<<" "<<height<<endl<<endl;
    
    for (int i = 0; i < height; i++){
        for (int j = 0; j < width; j++){
            outputFile<<!outputImage[i][j]<<" ";
        }
        outputFile<<endl;
    }
}

void convert_using_thresholding(void){
    
    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
            outputImage[i][j] = (float)(inputImage[i][j] > 0.5);
            
}

void convert_using_dithering(void){
    
    // please use at least dithering matrix of size 3x3
    // fill in the code here ...
    float dither[3][3] = {
        {0.0, 0.2, 0.5},
        {0.8, 0.4, 0.1},
        {0.6, 0.3, 0.7}
    };

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float oldVal = inputImage[y][x];
            float newVal = (oldVal > dither[y % 2][x % 3]) ? 1.0 : 0.0;
            outputImage[y][x] = newVal;
            float error = oldVal - newVal;

            // Distribute the error to neighboring pixels
            if (x + 1 < width) {
                inputImage[y][x + 1] += (5.0 / 48.0) * error;
            }
            if (x + 2 < width) {
                inputImage[y][x + 2] += (3.0 / 48.0) * error;
            }
            if (y + 1 < height) {
                if (x - 2 >= 0) {
                    inputImage[y + 1][x - 2] += (2.0 / 48.0) * error;
                }
                if (x - 1 >= 0) {
                    inputImage[y + 1][x - 1] += (4.0 / 48.0) * error;
                }
                inputImage[y + 1][x] += (5.0 / 48.0) * error;
                if (x + 1 < width) {
                    inputImage[y + 1][x + 1] += (4.0 / 48.0) * error;
                }
                if (x + 2 < width) {
                    inputImage[y + 1][x + 2] += (2.0 / 48.0) * error;
                }
            }
            if (y + 2 < height) {
                if (x - 2 >= 0) {
                    inputImage[y + 2][x - 2] += (1.0 / 48.0) * error;
                }
                if (x - 1 >= 0) {
                    inputImage[y + 2][x - 1] += (2.0 / 48.0) * error;
                }
                inputImage[y + 2][x] += (3.0 / 48.0) * error;
                if (x + 1 < width) {
                    inputImage[y + 2][x + 1] += (2.0 / 48.0) * error;
                }
                if (x + 2 < width) {
                    inputImage[y + 2][x + 2] += (1.0 / 48.0) * error;
                }
            }
        }
    }
}


void convert_using_error_diffusion(void){
    
    // fill in the code here ...
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float old_pixel = inputImage[y][x];

            float new_pixel = old_pixel > 0.5 ? 1.0f : 0.0f;
            outputImage[y][x] = new_pixel;

            float error = old_pixel - new_pixel;

            if (x + 1 < width) {
                inputImage[y][x + 1] += error * 7.0f / 16.0f;
            }
            if (y + 1 < height) {
                if (x - 1 >= 0) {
                    inputImage[y + 1][x - 1] += error * 3.0f / 16.0f;
                }
                inputImage[y + 1][x] += error * 5.0f / 16.0f;
                if (x + 1 < width) {
                    inputImage[y + 1][x + 1] += error * 1.0f / 16.0f;
                }
            }
        }
    }
}


int main(int argc, char **argv) {

    if(argc < 4){
        cout<<"Usage: Halftoning IntputImageFileName OutputImageFileName Method"<<endl;
        cout<<"Method = Thresholding | Dithering | ErrorDiffusion"<<endl;
        return 0;
    }
    
    read_image(argv[1]);
    
    if (strcmp(argv[3],"Thresholding") == 0) convert_using_thresholding();
    else{
        if (strcmp(argv[3],"Dithering") == 0) convert_using_dithering();
        else{
            if (strcmp(argv[3],"ErrorDiffusion") == 0) convert_using_error_diffusion();
            else{
                cout<<"Usage: Halftoning IntputImageFileName OutputImageFileName Method"<<endl;
                cout<<"Method = Thresholding | Dithering | ErrorDiffusion"<<endl;
                return 0;
            }
        }
    }
    
    write_image(argv[2]);
    
    return 0;
}
