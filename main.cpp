#include <ctime>
#include <iostream>
#include <fstream>
#include <math.h>

using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.hpp>


int main()
{
    time_t tstart, tend;
    tstart = time(0);
    
    int SIZE = 31303; // from data set
    float DISTANCE = 20; // NN distance
    float sigma = 30;
    float tolerance = 0.000001;
    
    ifstream in("intput.dat");
    ofstream out("output.dat");
    
    float* x = new float[SIZE];
    float* y = new float[SIZE];
    float* z = new float[SIZE];
    
    float* x2 = new float[SIZE];
    float* y2 = new float[SIZE];
    float* z2 = new float[SIZE];
    
    for (int i = 0; i<SIZE; i++)
    {
        float dummy;
        in>>x[i]>>y[i]>>z[i]>>dummy;
        in>>x2[i]>>y2[i]>>z2[i]>>dummy; //line segament input
        
    }
    
    
    for (int i = 0; i<SIZE; i++)
    {
        cv::Mat S = cv::Mat::zeros(3, 3, CV_32F);
        for(int j =0; j<SIZE; j++)
        {
            float diffx, diffy, diffz;
            diffx = x[i] - x[j];
            diffy = y[i] - y[j];
            diffz = z[i] - z[j];
            float d = diffx*diffx + diffy*diffy + diffz*diffz;
            if(d<DISTANCE && d>tolerance)
            {
                float c = exp(d/sigma);
                d = sqrt(d);
                float rVec[1][3] = {{diffx/d, diffy/d, diffz/d}};
                cv::Mat r = cv::Mat(3, 1, CV_32F, rVec);
                // ========= point cloud case =========
                //cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
                // ====================================
                
                // ======= line segaments case ========
                float dx, dy, dz;
                dx = x2[j] - x[j];
                dy = y2[j] - y[j];
                dz = z2[j] - z[j];
                float d = diffx*diffx + diffy*diffy + diffz*diffz;
                d = sqrt(d);
                float KVec[1][3] = {{diffx/d, diffy/d, diffz/d}};
                cv::Mat temp = cv::Mat(3, 1, CV_32F, KVec);
                cv::Mat K = cv::Mat::zeros(3, 3, CV_32F);
                K = cv::Mat::eye(3, 3, CV_32F) - temp * temp.t();
                // ====================================
                
                cv::Mat R = K - 2 * r * r.t();
                cv::Mat R2 = (K - 0.5 * r * r.t()) * R;
                
                S = S + c * R * K * R2;
            }
            
        }
        cv::Mat Ev =cv::Mat::zeros(3, 3, CV_32F); // Eigenvectors
        cv::Mat v = cv::Mat::zeros(3, 1, CV_32F); // Eigenvalues
        eigen(S, v, Ev);                          // requires S to be symmetric matrix
        
        float diffLambda = v.at<float>(0,0) - v.at<float>(0,1);
        diffLambda = diffLambda/5; // for plot usage
        out<<x[i]<<" "<<y[i]<<" "<<z[i]<<" 0"<<endl;
        out<<x[i]+diffLambda*Ev.at<float>(0,0)<<" "<<y[i]+diffLambda*Ev.at<float>(0,1)<<" "<<z[i]+diffLambda*Ev.at<float>(0,2)<<" 1"<<endl;
        
        //if(diffLambda>3)  // output point, for Matlab plot
        //{
        //    out<<x[i]<<" "<<y[i]<<" "<<z[i]<<" "<<diffLambda<<endl;
        //}
    }
    
    tend = time(0);
    
    cout<<difftime(tend, tstart)<<" "<<endl;
    
    
    
    
    delete [] x;
    delete [] y;
    delete [] z;
    
    delete [] x2;
    delete [] y2;
    delete [] z2;
}

