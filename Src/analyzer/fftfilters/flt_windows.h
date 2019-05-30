#ifndef FLT_WINDOW_H_INCLUDED
#define FLT_WINDOW_H_INCLUDED

/* --- windowing --- */
__inline double w_hann(const double pos)
{
	double x = cos(pos*PI/2.0);
	return x*x;
}

__inline double w_rect(const double pos)
{
	return 1.0;
}

__inline double w_hamming(const double pos)
{
	return 0.54 + 0.46 * cos(PI*pos);
}

__inline double w_blackman(const double pos)
{
	return 0.42 + 0.5 * cos(PI*pos) + 0.08 * cos(2.0*PI*pos);
}

__inline double w_gaussian(const double pos)
{
	double a = 4.0;
	double ax = a * pos;
	return exp(-0.5 * ax*ax);
}

__inline double w_welch(const double pos)
{
	return 1.0 - pos*pos;
}

__inline double w_bartlett(const double pos)
{
	if (pos == 0.0)
		return 1.0;
	else if (pos > -1.0 && pos < 0.0)
		return pos + 1.0;
	else if (pos > 0.0 && pos < 1.0)
		return 1.0 - pos;
	else
		return 0.0;
}


#define WINDOW_FUNC	w_gaussian
/*
void LoadWindowData()
{
    for (int i = 0; i < FFT_AUDIO_SIZE; i++)
    {
        windowData[i] = (WINDOW_FUNC((double)(i-(double)FFT_AUDIO_SIZE/2) / (double)((double)FFT_AUDIO_SIZE/2)));
    }
}
*/

#endif //FILT_WINDOW
