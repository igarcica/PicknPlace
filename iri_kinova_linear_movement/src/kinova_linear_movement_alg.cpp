#include "kinova_linear_movement_alg.h"

KinovaLinearMovementAlgorithm::KinovaLinearMovementAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

KinovaLinearMovementAlgorithm::~KinovaLinearMovementAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void KinovaLinearMovementAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// KinovaLinearMovementAlgorithm Public API
