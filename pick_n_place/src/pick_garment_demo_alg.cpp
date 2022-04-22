#include "pick_garment_demo_alg.h"

PickGarmentDemoAlgorithm::PickGarmentDemoAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

PickGarmentDemoAlgorithm::~PickGarmentDemoAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void PickGarmentDemoAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// PickGarmentDemoAlgorithm Public API
