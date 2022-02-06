#include "utils.h"

float round(float num, int decimals)
{
    float factor = pow(10, decimals);
    return floor(num * factor + 0.5) / factor;
}
