#include "bundleadjustinf.h"
#include "bundleadjust.h"
#include "bundleadjust2.h"
BundleAdjustInf * BundleAdjustInf::create_instance(int method)
{
    switch (method) {
    case 1:
        return new BundleAdjust();
    case 2:
        return new BundleAdjust2();
	case 0:
		return new BundleAdjust2();
    }
    return NULL;
}
