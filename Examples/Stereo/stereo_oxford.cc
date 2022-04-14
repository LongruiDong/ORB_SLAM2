/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include <include/Converter.h>//用来进行数据转换

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);
//从表示图像和点云对齐的associations.txt来读入数据
void LoadImagefromAsat(const string &strAssociationFilename, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps,
                const int &start, const int &end);

int main(int argc, char **argv)
{
    if(argc != 7)
    {
        cerr << endl << "Usage: ./stereo_oxford path_to_vocabulary path_to_settings path_to_sequence subid start end" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[3])+"/associations.txt";
    int startf = stoi(argv[5]);
    int endf = stoi(argv[6]);
    string pathtoseq=string(argv[3]); // /home/dlr/kitti/dataset/sequences/04
    string seqid = string(argv[4]);//pathtoseq.substr(pathtoseq.length()-2); // 04
    // LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);
    LoadImagefromAsat(strAssociationFilename, vstrImageLeft, vstrImageRight, vTimestamps, startf, endf);

    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,false);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sub sequence oxford "<<seqid<<"... from "<< pathtoseq << endl;
    cout << "Images in the sub sequence: " << nImages << endl << endl;   

    string ftrack = "result/stereotrack/"   //记录前端的tracking结果 验证是否和输入的初始轨迹一样
                        + seqid +".txt";
    ofstream f;
    f.open(ftrack.c_str());
    f << fixed;
    // Main loop
    cv::Mat imLeft, imRight, rawimLeft, rawimRight;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        rawimLeft = cv::imread(string(argv[3])+"/"+vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        rawimRight = cv::imread(string(argv[3])+"/"+vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        int rawW = rawimLeft.cols;
        int rawH = rawimLeft.rows;
        int newOw = 0;
        int newOh = 0;
        int newW = rawW;
        int newH = rawH-160;
        // 对oxford图像 W,H 1280 960 进行裁剪 主要为了去除视野内车体
        cv::Rect rect(newOw, newOh, newW, newH);
        imLeft = rawimLeft(rect);
        imRight = rawimRight(rect);
        if(ni==0)
        {
            cout<<"raw image size: "<<rawW<<", "<<rawH<<endl;
            cout<<"rect("<<newOw<<", "<<newOh<<", "<<newW<<", "<<newH<<")"<<endl;
            cout<<"new image size: "<<imLeft.cols<<", "<<imLeft.rows<<endl;
        }
        // cv::imwrite("Rawimageleft_x.png", rawimLeft);
        // cv::waitKey(0);
        // cv::destroyAllWindows();
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        cv::Mat T_track = SLAM.TrackStereo(imLeft,imRight,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
        if(T_track.empty())//合法性检查
        {
            cerr << endl << "WARNING: Empty track pose at: "
                 << ni << endl;
            return 1;
        }
        cv::Mat Ttrack = ORB_SLAM2::Converter::InverseMat(T_track.clone());//取逆得到Twc
        //输出到文件 保存为kitti轨迹格式 小数点后6位小数
        f << setprecision(9) << Ttrack.at<float>(0, 0) << " " << Ttrack.at<float>(0, 1) << " " << Ttrack.at<float>(0, 2) << " " << Ttrack.at<float>(0, 3) << " " 
                             << Ttrack.at<float>(1, 0) << " " << Ttrack.at<float>(1, 1) << " " << Ttrack.at<float>(1, 2) << " " << Ttrack.at<float>(1, 3) << " " 
                             << Ttrack.at<float>(2, 0) << " " << Ttrack.at<float>(2, 1) << " " << Ttrack.at<float>(2, 2) << " " << Ttrack.at<float>(2, 3) << endl;
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }
    f.close();
    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    string savepath = "result/stereo/" + seqid +".txt";
    SLAM.SaveTrajectoryKITTI(savepath);

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}

void LoadImagefromAsat(const string &strAssociationFilename, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps,
                const int &start, const int &end)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    int numd = 0;//记录数据id 
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t,tin;
            string sRGB, sScan;
            ss >> t;
            if (numd>=start && numd<=end)
            {
                tin = double(t)/double(1000000.0);//化为以s为单位
                // cerr<<setprecision(6)<<fixed<<tin<<endl;
                vTimestamps.push_back(tin);
                ss >> sRGB;
                //左右图的名字时间相同
                vstrImageLeft.push_back("stereo/left_rect/"+sRGB);
                vstrImageRight.push_back("stereo/right_rect/"+sRGB);
                // ss >> t;
                // ss >> sScan; // _mcins 使用ins pose 补偿
                // vstrScan.push_back("velodyne_right/"+sScan); //velodyne_right_mc 是已经补偿过的点云
            }
            numd++;//计数
            if (numd>end)
            {
                break;
            }
        }
    }
    const int nTimes = vTimestamps.size();//总帧数
    if (nTimes!=end-start+1)
    {
        cerr<<"load data size ERROR: nTimes "<<nTimes<<" != end-start+1 "
        <<end-start+1<<" !"<<endl;
    }
}
