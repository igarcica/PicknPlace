#include "pick_n_place_alg.h"

PicknPlaceAlgorithm::PicknPlaceAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

PicknPlaceAlgorithm::~PicknPlaceAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void PicknPlaceAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// PicknPlaceAlgorithm Public API
