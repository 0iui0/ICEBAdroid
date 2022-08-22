#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <sys/types.h>
#include <dirent.h>
#include <vector>
#include <string>
using namespace std;
using namespace cv;

void GetFileNames(string path,vector<string>& filenames)
{
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str()))){
        cout<<"Folder doesn't Exist!"<<endl;
        return;
    }
    while((ptr = readdir(pDir))!=0) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0){
            filenames.push_back(path + "/" + ptr->d_name);
    }
    }
    closedir(pDir);
}

void calibrFishEye(string folder){

	vector<string> file_name;
	GetFileNames(folder, file_name);
	cout << "image folder "<<folder<<" contain "<< file_name.size()<<" image"<< endl;
	if(file_name.size()==0)
		return;


	string resultN=folder.substr(folder.size()-9,4)+"result.yml";
	FileStorage fs(resultN.c_str(), FileStorage::WRITE);

	/************************************************************************
	读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化
	*************************************************************************/
	cout << "开始提取角点………………" << endl;
	int image_count = file_name.size();                    /****    图像数量     ****/
	Size board_size = Size(9, 6);            /****    定标板上每行、列的角点数       ****/
	vector<Point2f> corners;                  /****    缓存每幅图像上检测到的角点       ****/
	vector<vector<Point2f>>  corners_Seq;    /****  保存检测到的所有角点       ****/
	vector<Mat>  image_Seq;
	int successImageNum = 0;                /****   成功提取角点的棋盘图数量    ****/

	int count = 0;
	for (int i = 0; i != file_name.size(); i++)
	{
		cout << "Frame #" << i + 1 << "..." << endl;

		cv::Mat imageGray = imread(file_name[i],-1);
		/* 提取角点 */
		bool patternfound = findChessboardCorners(imageGray, board_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE +
			CALIB_CB_FAST_CHECK);
		if (!patternfound)
		{
			cout << file_name[i]<<" 找不到角点" << endl;
			image_count--;
		}
		else
		{
			/* 亚像素精确化 */
			cornerSubPix(imageGray, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));
			/* 绘制检测到的角点并保存 */
			Mat imageTemp = imageGray.clone();
			for (int j = 0; j < corners.size(); j++)
			{
				circle(imageTemp, corners[j], 10, Scalar(0, 0, 255), 2, 8, 0);
			}
			cv::imshow("corner",imageTemp);
			cv::waitKey(1);
			count = count + corners.size();
			successImageNum ++;
			corners_Seq.push_back(corners);
			image_Seq.push_back(imageGray);
			cout << "image_Seq size"<<image_Seq.size() << endl;
		}
	}
	cout << "角点提取完成！\n";
	/************************************************************************
	摄像机定标
	*************************************************************************/
	cout << "开始定标………………" << endl;
	Size square_size = Size(40, 40);
	vector<vector<Point3f>>  object_Points;        /****  保存定标板上角点的三维坐标   ****/

	Mat image_points = Mat(1, count, CV_32FC2, Scalar::all(0));  /*****   保存提取的所有角点   *****/
	vector<int>  point_counts;
	/* 初始化定标板上角点的三维坐标 */
	for (int t = 0; t<successImageNum; t++)
	{
		vector<Point3f> tempPointSet;
		for (int i = 0; i<board_size.height; i++)
		{
			for (int j = 0; j<board_size.width; j++)
			{
				/* 假设定标板放在世界坐标系中z=0的平面上 */
				Point3f tempPoint;
				tempPoint.x = i*square_size.width;
				tempPoint.y = j*square_size.height;
				tempPoint.z = 0;
				tempPointSet.push_back(tempPoint);
			}
		}
		object_Points.push_back(tempPointSet);
	}
	for (int i = 0; i< successImageNum; i++)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}
	/* 开始定标 */
	Size image_size = image_Seq[0].size();
	cv::Matx33d intrinsic_matrix;    /*****    摄像机内参数矩阵    ****/
	cv::Vec4d distortion_coeffs;     /* 摄像机的4个畸变系数：k1,k2,k3,k4*/
	std::vector<cv::Vec3d> rotation_vectors;                           /* 每幅图像的旋转向量 */
	std::vector<cv::Vec3d> translation_vectors;                        /* 每幅图像的平移向量 */
	int flags = 0;
	flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
	flags |= cv::fisheye::CALIB_CHECK_COND;
	flags |= cv::fisheye::CALIB_FIX_SKEW;
	cout << "object_Points size "<<object_Points.size()<<endl;
	cout << "corners_Seq size "<<corners_Seq.size()<<endl;
	fisheye::calibrate(object_Points, corners_Seq, image_size, intrinsic_matrix, distortion_coeffs, rotation_vectors, translation_vectors, flags, cv::TermCriteria(3, 20, 1e-6));
	cout << "定标完成！\n";

	/************************************************************************
	对定标结果进行评价
	*************************************************************************/
	cout << "开始评价定标结果………………" << endl;
	double total_err = 0.0;                   /* 所有图像的平均误差的总和 */
	double err = 0.0;                        /* 每幅图像的平均误差 */
	vector<Point2f>  image_points2;             /****   保存重新计算得到的投影点    ****/

	cout << "每幅图像的定标误差：" << endl;
	cout << "每幅图像的定标误差：" << endl << endl;
	for (int i = 0; i<image_count; i++)
	{
		vector<Point3f> tempPointSet = object_Points[i];
		/****    通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点     ****/
		fisheye::projectPoints(tempPointSet, image_points2, rotation_vectors[i], translation_vectors[i], intrinsic_matrix, distortion_coeffs);
		/* 计算新的投影点和旧的投影点之间的误差*/
		vector<Point2f> tempImagePoint = corners_Seq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (size_t i = 0; i != tempImagePoint.size(); i++)
		{
			image_points2Mat.at<Vec2f>(0, i) = Vec2f(image_points2[i].x, image_points2[i].y);
			tempImagePointMat.at<Vec2f>(0, i) = Vec2f(tempImagePoint[i].x, tempImagePoint[i].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += err /= point_counts[i];
		cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
	}
	cout << "总体平均误差：" << total_err / image_count << "像素" << endl;
	cout << "评价完成！" << endl;

	/************************************************************************
	保存定标结果
	*************************************************************************/
	cout << "开始保存定标结果………………" << endl;
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */

    if( fs.isOpened() )
    {
        fs << "intrinsic_matrix" << intrinsic_matrix << "distortion_coeffs" << distortion_coeffs;
        fs.release();
    }

	cout << "完成保存" << endl;


	/************************************************************************
	显示定标结果
	*************************************************************************/
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);

	cout << "保存矫正图像" << endl;
	for (int i = 0; i != image_count; i++)
	{
		cout << "Frame #" << i + 1 << "..." << endl;
		//fisheye::initUndistortRectifyMap(intrinsic_matrix,distortion_coeffs,R,intrinsic_matrix,image_size,CV_32FC1,mapx,mapy);
		fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R,
			getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coeffs, image_size, 1, image_size, 0), image_size, CV_32FC1, mapx, mapy);
		Mat t = image_Seq[i].clone();
		cv::remap(image_Seq[i], t, mapx, mapy, INTER_LINEAR);

		cv::imshow("undistor",t);
		cv::waitKey(10);
	}
	cout << "保存结束" << endl;


	/************************************************************************
	测试一张图片
	*************************************************************************/
	if (1)
	{
		//cout<<"TestImage ..."<<endl;
		//Mat testImage = imread("a.jpg",1);
		//fisheye::initUndistortRectifyMap(intrinsic_matrix,distortion_coeffs,R,
		//    getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coeffs, image_size, 1, image_size, 0),image_size,CV_32FC1,mapx,mapy);
		//Mat t = testImage.clone();
		//cv::remap(testImage,t,mapx, mapy, INTER_LINEAR);

		//imwrite("TestOutput.jpg",t);
		//cout<<"保存结束"<<endl;

		cout << "TestImage ..." << endl;
        for (size_t i = 0; i < image_Seq.size(); i++)
        {
		Mat distort_img = image_Seq[i];
		Mat undistort_img;
		Mat intrinsic_mat(intrinsic_matrix), new_intrinsic_mat;

		intrinsic_mat.copyTo(new_intrinsic_mat);
		//调节视场大小,乘的系数越小视场越大
		new_intrinsic_mat.at<double>(0, 0) *= 0.5;
		new_intrinsic_mat.at<double>(1, 1) *= 0.5;
		//调节校正图中心，建议置于校正图中心
		new_intrinsic_mat.at<double>(0, 2) = 0.5 * distort_img.cols;
		new_intrinsic_mat.at<double>(1, 2) = 0.5 * distort_img.rows;

		fisheye::undistortImage(distort_img, undistort_img, intrinsic_matrix, distortion_coeffs, new_intrinsic_mat);
		cv::imshow("undistort_img",undistort_img);
		cv::waitKey(1000);
        }

	}
}

int main(int argc, char **argv)
{
	
    string path = "/home/cy/data/GvDevice/fish_chess";
	if(argc>1)
		path=std::string(argv[1]);
    string folder0=path+"/cam0/data";
    string folder1=path+"/cam1/data";

	calibrFishEye(folder0);
	calibrFishEye(folder1);

	return 0;
}

// #include <vector>
// #include <string>
// #include <algorithm>
// #include <iostream>
// #include <iterator>
// #include <stdio.h>
// #include <stdlib.h>
// #include <ctype.h>

// using namespace cv;
// using namespace std;

// static void fisheyeStereoCalib(const vector<string>& imageList, const vector<Mat>& pre_calib, Size& boardSize, float squareSize, bool showRectified=true){
//     if( imageList.size() % 2 != 0 )
//     {
//         cout << "Error: the image list contains odd (non-even) number of elements\n";
//         return;
//     }

//     // ARRAY AND VECTOR STORAGE:

//     vector<vector<Point2f> > imagePoints[2];
//     vector<vector<Point3f> > objectPoints;
//     Size imageSize;

//     int i, j, k, n_images = (int)imageList.size()/2;

//     imagePoints[0].resize(n_images);
//     imagePoints[1].resize(n_images);
//     vector<string> goodImageList;

//     cout<<"Press the blank space if you think the corners are detected correctly. Otherwise press any other keys to continue."<<endl;

//     for( i = j = 0; i < n_images; i++ )
//     {
//         for( k = 0; k < 2; k++ )
//         {
//             const string& filename = imageList[i*2+k];
//             Mat img = imread(filename, 0);
//             if(img.empty())
//                 break;
//             if( imageSize == Size() )
//                 imageSize = img.size();
//             else if( img.size() != imageSize )
//             {
//                 cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
//                 break;
//             }
//             bool found = false;
//             vector<Point2f>& corners = imagePoints[k][j];

//             found = findChessboardCorners(img, boardSize, corners,
//                                           CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

//             //display
//             {
//                 Mat c_img, c_img1;
//                 cvtColor(img, c_img, COLOR_GRAY2BGR);
//                 drawChessboardCorners(c_img, boardSize, corners, found);
//                 double sf = 640./MAX(img.rows, img.cols);
//                 resize(c_img, c_img1, Size(), sf, sf, INTER_LINEAR_EXACT);
//                 imshow("corners"+to_string(k), c_img1);
//             }
//             if( !found )
//                 break;
//             cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
//                          TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
//                                       30, 0.01));
//         }
//         if( k == 2 )
//         {
//             int isOK=waitKey(0);
//             //press the blank space to validate
//             if(isOK==32){
//                 goodImageList.push_back(imageList[i*2]);
//                 goodImageList.push_back(imageList[i*2+1]);
//                 j++;
//             }
//             if(isOK==27)
//                 return;
//         }
//     }
//     cout << j << " pairs have been successfully detected.\n";
//     n_images = j;
//     if( n_images < 2 )
//     {
//         cout << "Error: too little pairs to run the calibration\n";
//         return;
//     }

//     imagePoints[0].resize(n_images);
//     imagePoints[1].resize(n_images);
//     objectPoints.resize(n_images);

//     for( i = 0; i < n_images; i++ )
//     {
//         for( j = 0; j < boardSize.height; j++ )
//             for( k = 0; k < boardSize.width; k++ )
//                 objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
//     }

//     cout << "Running single calibration first...\n";

//     //single fisheye calibration
//     Mat cameraMatrix[2], distCoeffs[2];
//     Mat rvecs[2], tvecs[2];

//     if(!pre_calib.empty()&&pre_calib.size()==4){
//         cameraMatrix[0]=pre_calib[0];
//         distCoeffs[0]=pre_calib[1];
//         cameraMatrix[1]=pre_calib[2];
//         distCoeffs[1]=pre_calib[3];

//         fisheye::calibrate(objectPoints,imagePoints[0],imageSize,cameraMatrix[0],distCoeffs[0],rvecs[0],tvecs[0],
//                            fisheye::CALIB_USE_INTRINSIC_GUESS+fisheye::CALIB_FIX_SKEW);
//         fisheye::calibrate(objectPoints,imagePoints[1],imageSize,cameraMatrix[1],distCoeffs[1],rvecs[1],tvecs[1],
//                            fisheye::CALIB_USE_INTRINSIC_GUESS+fisheye::CALIB_FIX_SKEW);
//     }
//     else{
//         fisheye::calibrate(objectPoints,imagePoints[0],imageSize,cameraMatrix[0],distCoeffs[0],rvecs[0],tvecs[0],
//                            fisheye::CALIB_FIX_SKEW);
//         fisheye::calibrate(objectPoints,imagePoints[1],imageSize,cameraMatrix[1],distCoeffs[1],rvecs[1],tvecs[1],
//                            fisheye::CALIB_FIX_SKEW);
//     }

//     cout<<"Intrinsic Matrix of left:\n"<<cameraMatrix[0]<<endl;
//     cout<<"Distortion Matrix of left:\n"<<distCoeffs[0]<<endl;
//     cout<<"Intrinsic Matrix of right:\n"<<cameraMatrix[1]<<endl;
//     cout<<"Distortion Matrix of right:\n"<<distCoeffs[1]<<endl;

//     // save intrinsic parameters
//     FileStorage fs("intrinsics.yml", FileStorage::WRITE);
//     if( fs.isOpened() )
//     {
//         fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
//            "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
//         fs.release();
//     }
//     else
//         cout << "Error: can not save the intrinsic parameters\n";

//     cout << "Running stereo calibration ...\n";

//     //stereo fisheye calibration
//     Mat R, T;
//     double rms=fisheye::stereoCalibrate(objectPoints,imagePoints[0],imagePoints[1],cameraMatrix[0],distCoeffs[0],
//             cameraMatrix[1],distCoeffs[1],imageSize,R,T);

//     cout<<"Extrinsic error (RMS):"<<rms<<endl;
//     //cout<<"Extrinsic Matrix:\nRotation Matrix:\n"<<R<<"\nTranslation Matrix:\n"<<T<<endl;

//     Mat R1, R2, P1, P2, Q;

//     fisheye::stereoRectify(cameraMatrix[0],distCoeffs[0],cameraMatrix[1],distCoeffs[1],imageSize,
//             R,T,R1,R2,P1,P2,Q,CALIB_ZERO_DISPARITY,imageSize);

//     cout<<"Q:\n1 0 0 -c_x\n0 1 0 -c_y\n0 0 0 f\n0 0 -1/T_x (c_x-c'_x)/T_x\n\n"<<Q<<endl;
//     cout<<"baseline: "<<1/Q.at<double>(3,2)<<endl;

//     fs.open("extrinsics.yml", FileStorage::WRITE);
//     if( fs.isOpened() )
//     {
//         fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
//         fs.release();
//     }
//     else
//         cout << "Error: can not save the extrinsic parameters\n";

//     // COMPUTE AND DISPLAY RECTIFICATION
//     if( !showRectified )
//         return;

//     Mat r_map[2][2];

//     fisheye::initUndistortRectifyMap(cameraMatrix[0],distCoeffs[0],R1,P1,imageSize,CV_32F,r_map[0][0],r_map[0][1]);
//     fisheye::initUndistortRectifyMap(cameraMatrix[1],distCoeffs[1],R2,P2,imageSize,CV_32F,r_map[1][0],r_map[1][1]);

//     for( i = 0; i < n_images; i++ )
//     {
//         Mat rec[2];
//         for( k = 0; k < 2; k++ )
//         {
//             Mat img = imread(goodImageList[i*2+k], 0);
//             remap(img, rec[k], r_map[k][0], r_map[k][1], INTER_LINEAR);
//         }

//         Mat align;
//         hconcat(rec[0],rec[1],align);

//         cvtColor(align,align,cv::COLOR_GRAY2BGR);

//         for( j = 0; j < align.rows; j += 20 )
//             line(align, Point(0, j), Point(align.cols, j), Scalar(0, 255, 255), 1, 8);

//         double sf = 640./MIN(align.rows, align.cols);
//         resize(align, align, Size(), sf, sf, INTER_LINEAR_EXACT);

//         imshow("rectified", align);
//         int c = waitKey();
//         if( c == 27 || c == 'q' || c == 'Q' )
//             break;
//     }
// }

// int main(int argc, char** argv)
// {
//     if(argc!=5){
//         cout<<"usage: path_of_images(should have two sub folders 'cam0' and 'cam1') length_of_squares(mm) points_per_row points_per_col"<<endl;
//         return 1;
//     }

//     string image_path=argv[1];
//     if(image_path.back()!='/')
//         image_path.append("/");

//     float squareSize=strtof(argv[2], nullptr);

//     int cols=strtol(argv[3], nullptr,10);
//     int rows=strtol(argv[4], nullptr,10);
//     Size boardSize(cols,rows);

// 	// vector<string> file_name;
// 	// GetFileNames(image_path, file_name);

//     vector<string> imageList;
//     for(int i=0;i<34;++i){
//         imageList.push_back(image_path+"cam0/"+to_string(i)+".jpg");
//         imageList.push_back(image_path+"cam1/"+to_string(i)+".jpg");
//     }

//     //K1,D1,K2,D2
//     //pre-set paras, maybe provided by the manufacturers
//     //make it yourself!!!
//     vector<Mat> pre_calib;
//     pre_calib.push_back((Mat_<float>(3,3)<<495.4471360018053, 0, 606.3073724339264,
//                                            0, 494.0535667349407, 406.3837502093948,
//                                            0, 0, 1));
//     pre_calib.push_back((Mat_<float>(4,1)<<0.5529121780480518,
//                                            0.3197459950639739,
//                                            -0.8405562489098664,
//                                            0.4668200235897118));
//     pre_calib.push_back((Mat_<float>(3,3)<<487.0684594864165, 0, 626.3955180818867,
//                                            0, 486.9533501160483, 403.5345172057971,
//                                            0, 0, 1));
//     pre_calib.push_back((Mat_<float>(4,1)<<0.5748746843032233,
//                                            0.2298043301962883,
//                                            -0.7527926335516133,
//                                            0.441431345733051));

//     fisheyeStereoCalib(imageList,pre_calib, boardSize, squareSize, true);
//     return 0;
// }