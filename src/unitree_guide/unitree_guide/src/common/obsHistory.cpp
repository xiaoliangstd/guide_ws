#include "common/obsHistory.h"

obsHistory::obsHistory(int obs_history_length, int num_obs)
                                            :buffer_size(obs_history_length)
{
    historyBuffer.setZero(buffer_size, num_obs); // buffer_size is coputed by included_step * decimation.
}