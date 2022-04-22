// The "Square Detector" program.
// It loads several images sequentially and tries to find squares in
// each image

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include<dirent.h>

#include <iostream>

#include <fstream>

using namespace cv;
using namespace std;

static void help(const char* programName)
{

    cout <<
    "\nThis program will display the segmented cloth entered as input image\n"
    "and its contour in conjuntion with the template contour.\n"
    "In addition, it will print the file name, the total area of the segmented\n"
    "cloth and the total spreding error and the area and error for each zone \n"
    "of the garment (grasping zone and hanging zone)." << "\n" << endl;

}


int thresh = 50, N = 11;
const char* wndname = "Segmented image";
bool first = false;
vector<vector<Point>> first_cont;
Point first_ptupleft;
Point first_ptdownright;
vector<vector<Point>> first_filled_area;
int first_total_area;
int first_half_left_area;
int first_half_right_area;
std::ofstream data_file;

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
static double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}
// returns sequence of squares detected on the image.
static void findSquares( const Mat& image, vector<vector<Point> >& squares )
{
    squares.clear();

    Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    vector<vector<Point> > contours;

    // find squares in every color plane of the image
    for( int c = 0; c < 1; c++ )
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        for( int l = 0; l < 1; l++ )
        {
            l=1;
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
//                Canny(gray0, gray, 0, thresh, 5);
                Canny(gray0, gray, 0, 50, 3); //with aperture 3 we can detect big wrinkles
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, Mat(), Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/50;
            }
////////////////////////////////////////////////////////////////////

////////// FIND CONTOURS AND APPROXIMATE POLYGON ///////////////
            vector<vector<Point>> contours;
            vector<vector<Point>> contours0;
            vector<vector<Point>> siluette;
            vector<Vec4i> hierarchy;
            Mat cnt_img = Mat::zeros(600, 800, CV_8UC3);
            Mat slt_img = Mat::zeros(600, 800, CV_8UC3);
            Mat diff_img = Mat::zeros(600, 1600, CV_8UC3);
 
            findContours( gray, contours0, RETR_TREE, CHAIN_APPROX_SIMPLE);
   
            contours.resize(contours0.size());
            siluette.resize(contours0.size());

            vector<Point> approx;
            for( size_t i = 0; i < contours0.size(); i++ )
            {
                approxPolyDP(Mat(contours0[i]), siluette[i], 3, true); //Contours
                approxPolyDP(Mat(contours0[i]), contours[i], arcLength(contours0[i], true)*0.02, true); //Polygons

                drawContours(cnt_img, contours, -1, Scalar(128,255,255), -1);
                drawContours(slt_img, siluette, -1, Scalar(128,255,255), -1);
                cout << contours0.size() << endl;
                Point pt = contours0[i][0];
                double ptx = pt.x+20;
                double pty = pt.y+10;
                String str = to_string(i+1); //Enumerate nº contours in image
                putText(slt_img, str, Point(ptx,pty), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,255), 2, LINE_AA);
                //imshow("Siluettes: ", slt_img);
            }

////////////////////////////////////////////////////////////////////

///////////////////// FIND CORNERS //////////////////////////

            Point ptupleft, ptdownright, ptmidle;
            Point pt;
            double diff, sum, pre_diff, pre_sum;

            for (size_t j = 0; j < contours.size(); j++)
            {
                pt = contours[j][0];
                ptupleft = ptdownright = pt;
    
                pre_diff = contours[0][0].x - contours[0][0].y;
                pre_sum = contours[0][0].x + contours[0][0].y;
    
                for (size_t i = 0; i < contours[j].size(); i++)
                {
                    pt = contours[j][i];
    
                    diff = pt.x - pt.y;
                    sum = pt.x + pt.y;
                    if(pre_sum > sum)
                    {
                        pre_sum=sum;
                        ptupleft = pt;
                    }
                    if(pre_diff < diff)
                    {
                        pre_diff=diff;
                        ptdownright = pt;
                    }
                }

                //Midle point
//                ptmidle.x = (ptdownright.x-ptupleft.x)/2;
//                ptmidle.y = (ptdownright.y-ptupleft.y)/2;

                circle(cnt_img, ptupleft, 5, Scalar(255,0,0), CV_FILLED);
                circle(cnt_img, ptdownright, 5, Scalar(255,0,0), CV_FILLED);
                circle(slt_img, ptupleft, 5, Scalar(255,0,0), CV_FILLED);
                circle(slt_img, ptdownright, 5, Scalar(255,0,0), CV_FILLED);
//                circle(slt_img, ptmidle, 5, Scalar(0,255,0), CV_FILLED);
            }

//            imshow("Corners + contour: ", slt_img);


////////// COMPARE WITH FIRST SILUETTE ///////////////

        
            Mat comp_img2 = Mat::zeros(600, 800, CV_8UC3);
            Mat init_stp_img = Mat::zeros(600, 800, CV_8UC3);
            if(first)
            {
                first_cont = siluette; //Save first contours
                //polylines(init_stp_img, first_cont, true, Scalar(0,255,0));
                //imshow("Initial setup: ", init_stp_img);
            }

            drawContours(slt_img, siluette, -1, Scalar(128,255,255), -1);
            polylines(slt_img, first_cont, true, Scalar(0,255,0));
            imshow("Comparison: ", slt_img);
            first=false;



            //Rotated siluette
//            Mat comp_img2 = Mat::zeros(600, 800, CV_8UC3);
//            if(first)
//            {
//                first_ptupleft=rot_slt[0][0];
//                first_ptdownright=rot_slt[0][0];
//                first_cont = rot_slt[0]; //Save first contourn
//
//                //Save upper left corner
//                double pre_sum = first_cont[0].x+first_cont[0].y;
//                double pre_diff = first_cont[0].x-first_cont[0].y;
//                for (size_t i = 0; i < first_cont.size(); i++)
//                {
//                    double sum = first_cont[i].x + first_cont[i].y; 
//                    double diff = first_cont[i].x - first_cont[i].y; 
//                    if(pre_sum > sum)
//                    {
//                        first_ptupleft = first_cont[i]; //Save first upleftcorner
//                        pre_sum=sum;
//                    }
//                    if(pre_diff < diff)
//                    {
//                        first_ptdownright = first_cont[i]; //Save first upleftcorner
//                        pre_diff=diff;
//                    }
//                }
//                first_filled_area=siluette; //Saved filled area
//            }
//            // Compute half point of the contour
//            int length=first_ptdownright.x-first_ptupleft.x;
//            int half_point = first_ptupleft.x+(length/2);
//            circle(comp_img2, Point(half_point,100), 2, Scalar(0,0,255), CV_FILLED);
//
//            //Show first and current contour
//            polylines(comp_img2, first_cont, true, Scalar(128,255,255), 3);
//
//            //Translate contour to match corners:
//
//            //Get upper left corner of rotated contour
//            pre_sum = rot_slt[0][0].x+rot_slt[0][0].y;
//            ptupleft=rot_slt[0][0];
//            for (size_t i = 0; i < rot_slt[0].size(); i++)
//            {
//                double sum = rot_slt[0][i].x + rot_slt[0][i].y; 
//                if(pre_sum > sum)
//                {
//                    pre_sum=sum;
//                    ptupleft = rot_slt[0][i];
//                }
//            }
//            
//            vector<vector<Point>> transl_slt = rot_slt;
//            Point diff_xy2;
//            diff_xy2.x = first_ptupleft.x - ptupleft.x;
//            diff_xy2.y = first_ptupleft.y - ptupleft.y;
//            for( size_t i = 0; i < rot_slt[0].size(); i++ )
//            {
//               transl_slt[0][i].x = rot_slt[0][i].x + diff_xy2.x; 
//               transl_slt[0][i].y = rot_slt[0][i].y + diff_xy2.y;
//               //circle(comp_img2, transl_slt[0][i], 5, Scalar(255,0,0), CV_FILLED);
//            } 
//            polylines(comp_img2, transl_slt[0], true, Scalar(0,255,0), 3);
//            line(comp_img2, Point(half_point,100), Point(half_point,418), Scalar(0,0,255), 3);
//            //line(comp_img2, Point(half_point,170), Point(half_point,345), Scalar(0,0,255), 3);
//            //line(comp_img2, Point(half_point,95), Point(half_point,425), Scalar(0,0,255), 3);
//            imshow("Compare siluette", comp_img2);
//
//            Mat comLeft = comp_img2(Rect(0, 0, half_point, comp_img2.rows)); 
//            Mat compRight = comp_img2(Rect(half_point, 0, half_point, comp_img2.rows));
//            //imshow("Half left2", comLeft);
//            //imshow("Half Right2", compRight);

////////////////////////////////////////////////////////////////////

////////// COMPUTE AREA ///////////////
        
//            // Fill area
//            vector<vector<Point>> filled_area = transl_slt;
//            Mat filled_img = Mat::zeros(600, 800, CV_8UC1);
//            drawContours(filled_img, filled_area, -1, Scalar(122,255,255), -1);
//
//            //Divide in two zones
//            Mat halfLeft = filled_img(Rect(0, 0, half_point, filled_img.rows)); 
//            Mat halfRight = filled_img(Rect(half_point, 0, half_point, filled_img.rows));
//
//            // Compute area:
//
//            //Save area of success garment
//            if(first)
//            {
//                first_total_area = countNonZero(filled_img);
//                first_half_left_area = countNonZero(halfLeft);
//                first_half_right_area = countNonZero(halfRight);
//            }
//
//            int total_area = countNonZero(filled_img);
//            int half_left_area = countNonZero(halfLeft);
//            int half_right_area = countNonZero(halfRight);
//            double perc_total_area = (double)total_area/first_total_area;
//            double perc_left_area = (double)half_left_area/first_half_left_area;
//            double perc_right_area = (double)half_right_area/first_half_right_area;
//            
//            if(perc_total_area> 1.0)
//                perc_total_area=1.0;
//            if(perc_left_area> 1.0)
//                perc_left_area = 1.0;
//            if(perc_right_area > 1.0)
//                perc_right_area = 1.0;
//
//            //cout << "Template total area: " << first_total_area << endl;
//            cout << "Total area: " << total_area << " / Total error: " << 1-perc_total_area << endl;
//            //cout << "Total Left: " << first_half_left_area << endl;
//            cout << "GZ area: " << half_left_area << " / GZ error: " << 1-perc_left_area << endl;
//            //cout << "Total Right: " << first_half_right_area << endl;
//            cout << "HZ area: " << half_right_area << " / HZ error: " << 1-perc_right_area << endl;
//            cout << "-------------------" << endl;
       
////////////////////////////////////////////////////////////////////

////////// WRITE CSV  ///////////////

//            data_file << "," << 1-perc_total_area << "," << 1-perc_left_area << "," << 1-perc_right_area << "," << endl;


//            first=false;

        }
    }
}

int main(int argc, char** argv)
{
    // File where to save benchmark results
    data_file.open ("./data.csv");
    data_file << ",total_area, left_area, right_area," << endl;

    // Get images
    vector<cv::String> filenames;
    glob("./images/*.png", filenames, false);

    help(argv[0]);

    if( argc > 1)
    {
     filenames[0] =  argv[1];
     filenames[1] =  "0";

        printf("\nNumber Of Arguments Passed: %d",argc);
        printf("\n----Following Are The Command Line Arguments Passed----");
        for(int counter=0;counter<argc;counter++)
            printf("\nargv[%d]: %s",counter,argv[counter]);
    }

    // Use success image as template
    Mat image = imread(filenames[0], IMREAD_COLOR);
    resize(image, image, Size(706, 518));
    if( image.empty() )
    {
        cout << "Couldn't load " << filenames[0] << endl;
    }
    cout << "file: " << filenames[0] << endl;
    vector<vector<Point> > squares;
    findSquares(image, squares);

//    polylines(image, squares, true, Scalar(0, 255, 0), 3, LINE_AA);

    // Load other images
    for( int i = 0; i<filenames.size(); i=i+1)
    {
        Mat image = imread(filenames[i], IMREAD_COLOR);
        resize(image, image, Size(706, 518));
        if( image.empty() )
        {
            cout << "Couldn't load " << filenames[i] << endl;
            continue;
        }
        cout << "file: " << filenames[i] << endl;
        vector<vector<Point> > squares;
        findSquares(image, squares);

        polylines(image, squares, true, Scalar(0, 255, 0), 3, LINE_AA);
        imshow(wndname, image);

        int c = waitKey();
        if( c == 27 )
           break;
    }

    data_file.close();
    return 0;
}
