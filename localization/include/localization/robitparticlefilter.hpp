#ifndef ROBITPARTICLEFILTER_HPP
#define ROBITPARTICLEFILTER_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <ctime>
#include <opencv2/opencv.hpp>

#include "robitvision.hpp"

using namespace std;
using namespace cv;

typedef struct Sample_
{
    cv::Point               pos;
    double                  *dist;
    double                  *theta;

}Sample;

class RobitParticleFilter
{
public:
    /** constructors and destructor */
    RobitParticleFilter();
    RobitParticleFilter(int w, int h, int n_particles, int n_features);

    ~RobitParticleFilter();

public:
    /** static member functions
     *  @brief transforms 2D point about y = -x
      */
    static void transform(cv::Point&);

    /** public member functions */
    void addParticles(cv::Point);
    void addFeatures(cv::Point);
    void createSamples();
    const int calcWeights(const vector<ObjectPos>&, const cv::Point&);
    void clearWeights();

    const int update();

    const vector<cv::Point>& getParticles() const { return particles; }
    const vector<cv::Point>& getFeatures() const { return features; }
    const vector<cv::Point>& getTranstFeatures() const { return transFeatures; }
    const vector<vector<ObjectPos> >& getSamples() const { return samples; }

public:
    /** public member variables */


private:
    /** private member functions
     *  @brief translates reference frame
      */
    static void translate(cv::Point&, cv::Point);

private:
    /** private member variables */
    int                         fieldW;
    int                         fieldH;

    int                         numOfParticles;
    int                         numOfFeatures;

    bool*                       particleCheck;

    vector<cv::Point>           particles;
    vector<cv::Point>           features;
    vector<vector<ObjectPos> >  samples;

    vector<cv::Point>           transParticles;
    vector<cv::Point>           transFeatures;

    vector<double>              weights;
    vector<double>              cuWeights;

    cv::Mat                     probabiliy;
};

#endif // ROBITPARTICLEFILTER_HPP
