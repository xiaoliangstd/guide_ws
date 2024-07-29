#ifndef OBSHISTORY_H
#define OBSHISTORY_H

#include <Eigen/Core>
#include <vector>
#include <iostream>

class obsHistory
{
    public:
        obsHistory(int obs_history_length = 0, int num_obs = 0); //:buffer_size(obs_history_length);

        void reset(Eigen::Ref<Eigen::VectorXf> obs)
        {
            for (int i = 0; i < buffer_size; ++i)
            {
               historyBuffer.row(i)<<obs.transpose();
            }
        }

        void insert(Eigen::Ref<Eigen::VectorXf> obs) 
        {
            historyBuffer.block(0, 0, historyBuffer.rows() - 1, historyBuffer.cols()) = historyBuffer.block(1, 0, historyBuffer.rows() - 1, historyBuffer.cols());
            historyBuffer.bottomRows(1) = obs.transpose();
        }

        Eigen::VectorXf get_obs_vec()
        {
            return historyBuffer.reshaped<Eigen::RowMajor>();
        }

        Eigen::MatrixXf get_obs_mat()
        {
            return historyBuffer;
        }

    private:
        Eigen::MatrixXf historyBuffer; 
        int buffer_size;
};
#endif