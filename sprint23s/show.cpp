#include <opencv2/core/core.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main()
	{
		Mat image;
		image = imread("1.jpg",1);
		imshow("Image",image);

		waitKey(0);
		return 0;

	}
