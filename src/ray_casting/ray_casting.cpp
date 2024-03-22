#include "ray_casting/ray_casting.hpp"

IRayCasting::IRayCasting() {}

void IRayCasting::precomputeSensorModelTable(WeighingProfile_t aWeighingProfile, int aMaxMapRange)
{
    mSensorModelTableWidth = aMaxMapRange + 1;
    mSensorModelTable.resize(mSensorModelTableWidth * mSensorModelTableWidth);

    for (int d = 0; d < mSensorModelTableWidth; d++) {
        float norm = 0.0f;
        for (int r = 0; r < mSensorModelTableWidth; r++) {
            float prob = 0.0f;
            float z = float(r - d);
            prob += aWeighingProfile.z_hit *
                    std::exp(-(z * z) /
                             (2.0f * aWeighingProfile.sigma_hit * aWeighingProfile.sigma_hit)) /
                    (aWeighingProfile.sigma_hit * sqrtf(2.0f * M_PI));
            if (r < d) {
                prob += 2.0f * aWeighingProfile.z_short * (d - r) / float(d);
            }
            if (int(r) == int(aMaxMapRange)) {
                prob += aWeighingProfile.z_max;
            }
            if (r < int(aMaxMapRange)) {
                prob += aWeighingProfile.z_rand * 1.0f / float(aMaxMapRange);
            }
            norm += prob;
            mSensorModelTable[r * mSensorModelTableWidth + d] = prob;
        }
        for (int j = 0; j < mSensorModelTableWidth; j++) {
            mSensorModelTable[j * mSensorModelTableWidth + d] /= norm;
        }
    }
}

void IRayCasting::computeWeights(std::vector<adx::data::Particled>& aParticles,
                                 float* aRays,
                                 std::vector<float>& aCloud,
                                 Map_t& aMapDescriptor)
{
    int numParticles = aParticles.size();
    int numRays = aCloud.size();
    for (int i = 0; i < numParticles; ++i) {
        double w_temp = 1.0f;
        for (unsigned int j = 0; j < aCloud.size(); ++j) {
            float realRayPX = aCloud[j] / aMapDescriptor.map_resolution;
            realRayPX =
              std::min<float>(std::max<float>(realRayPX, 0.0), (aMapDescriptor.MAX_RANGE_PX - 1.0));
            float virtualRayPX = aRays[i * numRays + j] / aMapDescriptor.map_resolution;
            virtualRayPX = std::min<float>(std::max<float>(virtualRayPX, 0.0),
                                           (aMapDescriptor.MAX_RANGE_PX - 1.0));
            w_temp *=
              mSensorModelTable[int(virtualRayPX) * mSensorModelTableWidth + int(realRayPX)];
        }
        aParticles[i].weight = w_temp;
    }
}

void IRayCasting::setCloudDescriptor(Cloud_t& aCloudData)
{
    mCloudData = aCloudData;
}

void IRayCasting::weighParticles(std::vector<adx::data::Particled>& aParticles,
                                 std::vector<float>& aCloud,
                                 float* aDistMap,
                                 Map_t& aMapDescriptor,
                                 std::vector<float>& aRaysAngles)
{
    float* rays = new float[aParticles.size() * aCloud.size()];
    castRays(aParticles, aDistMap, aMapDescriptor, aRaysAngles, rays);
    computeWeights(aParticles, rays, aCloud, aMapDescriptor);
    delete[] rays;
}