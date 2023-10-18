#ifndef UWV_KALMAN_FILTERS_MAHALANOBIS
#define UWV_KALMAN_FILTERS_MAHALANOBIS

namespace uwv_kalman_filters
{
// functions for innovation gate test, using mahalanobis distance
template<typename scalar_type>
static bool d2p99(const scalar_type & mahalanobis2)
{
  if (mahalanobis2 >
    9.21)    // for 2 degrees of freedom, 99% likelihood = 9.21,
             // https://www.uam.es/personal_pdi/ciencias/anabz/Prest/Trabajos/Critical_Values_of_the_Chi-Squared_Distribution.pdf
  {
    return false;
  } else {
    return true;
  }
}

template<typename scalar_type>
static bool d2p95(const scalar_type & mahalanobis2)
{
  if (mahalanobis2 >
    5.991)    // for 2 degrees of freedom, 95% likelihood = 5.991,
              // https://www.uam.es/personal_pdi/ciencias/anabz/Prest/Trabajos/Critical_Values_of_the_Chi-Squared_Distribution.pdf
  {
    return false;
  } else {
    return true;
  }
}
}

#endif
