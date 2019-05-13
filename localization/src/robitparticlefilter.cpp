#include "../include/localization/robitparticlefilter.hpp"

/** constructors and destructor */
RobitParticleFilter::RobitParticleFilter()
    : numOfParticles(0), numOfFeatures(0)
{
    particles.clear();
    features.clear();
    samples.clear();
}

RobitParticleFilter::RobitParticleFilter(int w, int h, int n_particles, int n_features)
    : fieldW(w), fieldH(h), numOfParticles(n_particles), numOfFeatures(n_features)
{
    particles.clear();
    features.clear();
    samples.clear();

    particleCheck = new bool[fieldW * fieldH];

    srand(time(NULL));
}

RobitParticleFilter::~RobitParticleFilter()
{

}

/** static member functions */
void RobitParticleFilter::transform(cv::Point &pt)
{
    int temp = pt.x;
    pt.x = pt.y * (-1);
    pt.y = temp * (-1);
}

void RobitParticleFilter::translate(cv::Point &pt, cv::Point refPt)
{
    pt.x -= refPt.x;
    pt.y -= refPt.y;
}

/** public member functions */
void RobitParticleFilter::addParticles(cv::Point pastPt)
{
    int x_, y_;
    int fieldArea = fieldW * fieldH;

//    if(pastPt.x != 0 && pastPt.y != 0)
//    {
//        pastPt.x += fieldW / 2;
//        pastPt.y += fieldH / 2;
//    }

    int iteration = 0;
    int weightedN = 0;

    memset(particleCheck, false, sizeof(bool) * fieldW * fieldH);
    particles.clear();

    while(iteration < numOfParticles)
    {
        int randP = rand() % (fieldArea);
        if(!particleCheck[randP])
        {
            x_ = randP % fieldW;
            y_ = randP / fieldW;

            if(pastPt.x != 0 && pastPt.y != 0)
            {
                if(weightedN * 2 < numOfParticles)
                {
                    if(cv::norm(Point(x_, y_) - pastPt) < 100)
                    {
                        cv::Point pos(x_, y_);
                        particles.push_back(pos);
                        particleCheck[randP] = true;

                        this->transform(pos);
                        transParticles.push_back(pos);

                        iteration++;
                        weightedN++;
                    }
                }
                else if(cv::norm(Point(x_, y_) - pastPt) > 100)
                {
                    cv::Point pos(x_, y_);
                    particles.push_back(pos);
                    particleCheck[randP] = true;

                    this->transform(pos);
                    transParticles.push_back(pos);

                    iteration++;
                }
            }
            else
            {
                cv::Point pos(x_, y_);
                particles.push_back(pos);
                particleCheck[randP] = true;

                this->transform(pos);
                transParticles.push_back(pos);

                iteration++;
            }
        }
    }

//    std::cout << "particles : " << iteration << std::endl;
}

void RobitParticleFilter::addFeatures(Point feature)
{
    features.push_back(feature);

    this->transform(feature);
    transFeatures.push_back(feature);
}

void RobitParticleFilter::createSamples()
{
    samples = vector<vector<ObjectPos> >(particles.size());
    probabiliy = Mat::zeros(1, particles.size(), CV_64FC1);/*Mat(particles, CV_64FC1);*/

    for(size_t sp = 0; sp < particles.size(); sp++)
    {
        cv::Point temp = transParticles[sp];
        this->translate(transParticles[sp], temp);
        for(size_t sf = 0; sf < features.size(); sf++)
        {
            cv::Point temp1 = transFeatures[sf];
            this->translate(temp1, temp);
            ObjectPos sample;
            sample.dist = cv::norm(particles[sp] - features[sf]) * 10;
            sample.theta = atan2(temp1.y - transParticles[sp].y, temp1.x - transParticles[sp].x) * (180 / M_PI);

            samples[sp].push_back(sample);
        }
    }

    cuWeights.clear();
    cuWeights = vector<double>(particles.size());
}

const int RobitParticleFilter::calcWeights(const vector<ObjectPos> &obj, const Point &pastPt)
{
    weights.clear();

    int iteration = particles.size();
    int maxIdx = -1;
    double maxWeight = 0.0;

    for(int i = 0; i < iteration; i++)
    {
        double weight = 1.0;
        for(size_t s = 0; s < obj.size(); s++)
        {
            if(obj[s].dist != 0)
            {
                double distW = pow(abs(samples[i][s].dist - obj[s].dist), 2.0);
                double thtaW = pow(abs(samples[i][s].theta - obj[s].theta), 3.0);

                weight = weight * ((1 / distW) * (1 / thtaW)) * (double)(1 / (s + 1));
            }
        }

        if(pastPt.x != 0 && pastPt.y != 0)
        {
            double error = cv::norm(particles[i] - pastPt);
            double pastW = pow(error, 1.8);

            weight = weight * (1 / pastW);
        }

        weight += cuWeights[i];
        cuWeights[i] = weight;

        if(weight > maxWeight)
        {
            maxWeight = weight;
            maxIdx = i;
        }

        weights.push_back(weight);
    }

//    cout << "weight size: " << weights.size() << endl;
    return maxIdx;
}

void RobitParticleFilter::clearWeights()
{
    cuWeights.clear();
    cuWeights = vector<double>(particles.size());
}

const int RobitParticleFilter::update()
{
    int maxIdx = -1; double maxCWeight = 0.0;
    for(int i = 0; i < cuWeights.size(); i++)
    {
        if(cuWeights[i] > maxCWeight)
        {
            maxCWeight = cuWeights[i];
            maxIdx = i;
        }
    }

    return maxIdx;
}

/** private member functions */
