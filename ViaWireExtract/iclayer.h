#ifndef ICLAYER_H
#define ICLAYER_H

#include <vector>
#include <QImage>
#include <string>
using namespace std;

class ICLayerInterface
{
public:    
    virtual int getBlockWidth() =0;
	virtual void getBlockNum(int & bx, int &by) = 0;
    virtual int getRawImgByIdx(vector<uchar> & buff, int x, int y, int ovr, unsigned reserved) =0;
	virtual void putBlockNumWidth(int bx, int by, int width) = 0;
	virtual int addRawImg(vector<uchar> & buff, int x, int y, int ovr) = 0;
    virtual ~ICLayerInterface() {}
};

class ICLayerWr
{
public:
    ICLayerWr(string file, bool _read);
    ~ICLayerWr();
    int getBlockWidth();
	void getBlockNum(int & bx, int &by);
    int getRawImgByIdx(vector<uchar> & buff, int x, int y, int ovr, unsigned reserved);
	void generateDatabase(string path, int from_row, int to_row, int from_col, int to_col);
private:
    ICLayerInterface * layer_;
};

#endif // ICLAYER_H
