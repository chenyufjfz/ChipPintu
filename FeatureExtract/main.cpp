#include "featurewindow.h"
#include <QApplication>
#include "featext.h"
#include "opencv2/highgui/highgui.hpp"

int main(int argc, char** argv)
{
    FeatExt feature;
    ConfigPara cpara;

    cpara.clip_l = 6;
    cpara.clip_r = 0;
    cpara.clip_u = 0;
    cpara.clip_d = 12;
    cpara.rescale = 8;
    cpara.max_lr_xshift = 80;
    cpara.max_lr_yshift = 16;
    cpara.max_ud_xshift = 16;
    cpara.max_ud_yshift = 80;
    cpara.img_path = "F:/chenyu/work/ChipStitch/data/m3/";
    cpara.img_num_w = 71;
	cpara.img_num_h = 101;
    cpara.offset.create(cpara.img_num_h, cpara.img_num_w);
    for (int y = 0; y < cpara.img_num_h; y++) {
        for (int x = 0; x < cpara.img_num_w; x++) {
            cpara.offset(y, x)[1] = 1780 * x;
            cpara.offset(y, x)[0] = 1572 * y;
        }
    }
#if 1
	feature.set_cfg_para(cpara);
    feature.generate_feature_diff();
    feature.write_diff_file("diff.xml");
#else
	feature.read_diff_file("diff.xml");
#endif
    double minval, maxval;
    Point minloc, maxloc;
    minMaxLoc(feature.get_diff1(1, 0), &minval, &maxval, &minloc, &maxloc);
	minloc *= cpara.rescale;
    Mat img1 = imread(cpara.img_path + "2_1.jpg", 0);
    Mat img2 = imread(cpara.img_path + "2_2.jpg", 0);
    img1 = img1(Rect(cpara.clip_l, cpara.clip_u, img1.cols - cpara.clip_l - cpara.clip_r, img1.rows - cpara.clip_u - cpara.clip_d));
    img2 = img2(Rect(cpara.clip_l, cpara.clip_u, img2.cols - cpara.clip_l - cpara.clip_r, img2.rows - cpara.clip_u - cpara.clip_d));
	Point offset = feature.get_diff_offset1(1, 0) + minloc;
	Mat left = img1(Rect(offset.x, max(offset.y, 0), img1.cols - offset.x, img1.rows - abs(offset.y)));
	Mat right = img2(Rect(0, max(-offset.y, 0), img1.cols - offset.x, img1.rows - abs(offset.y)));
    resize(left, left, Size(left.cols / cpara.rescale, left.rows / cpara.rescale));
    resize(right, right, Size(right.cols / cpara.rescale, right.rows / cpara.rescale));
    imshow("x0", left);
    imshow("x1", right);

    minMaxLoc(feature.get_diff0(2, 1), &minval, &maxval, &minloc, &maxloc);
	minloc *= cpara.rescale;
    img1 = imread(cpara.img_path + "3_2.jpg", 0);
    img2 = imread(cpara.img_path + "4_2.jpg", 0);
    img1 = img1(Rect(cpara.clip_l, cpara.clip_u, img1.cols - cpara.clip_l - cpara.clip_r, img1.rows - cpara.clip_u - cpara.clip_d));
    img2 = img2(Rect(cpara.clip_l, cpara.clip_u, img2.cols - cpara.clip_l - cpara.clip_r, img2.rows - cpara.clip_u - cpara.clip_d));
	offset = feature.get_diff_offset0(2, 1) + minloc;
    Mat up = img1(Rect(max(offset.x, 0), offset.y, img1.cols - abs(offset.x), img1.rows - offset.y));
    Mat down = img2(Rect(max(-offset.x, 0), 0, img1.cols - abs(offset.x), img1.rows - offset.y));
    resize(up, up, Size(up.cols / cpara.rescale, up.rows / cpara.rescale));
    resize(down, down, Size(down.cols / cpara.rescale, down.rows / cpara.rescale));
    imshow("y0", up);
    imshow("y1", down);
    waitKey();

    return 0;
}

/*
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    FeatureWindow w;
    w.show();

    return a.exec();
}
*/
