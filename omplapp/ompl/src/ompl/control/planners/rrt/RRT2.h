
/* Author: John Noonan */

#ifndef OMPL_CONTROL_PLANNERS_RRT_RRT2_
#define OMPL_CONTROL_PLANNERS_RRT_RRT2_

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

namespace ompl
{
    namespace control
    {
        /**
         * Implementation of RRT2.0 which integrates kinodynamic constraints into an augmented state-cost space.
        */

        class RRT2 : public base::Planner
        {
        public:
            /** \brief Constructor */
            RRT2(const SpaceInformationPtr &si);

            ~RRT2() override;

            /** \brief Continue solving for some amount of time. Return true if solution was found. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /** \brief Clear the datastructures. */
            void clear() override;

            /** The goal bias. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Set the state weight for the nearest neighbors distance computation. */
            void setWx(double wx)
            {
                wx_ = wx;
            }

            /** \brief Get the state weight for the nearest neighbors distance computation. */
            double getWx() const 
            {
                return wx_;
            }

            /** \brief Set the cost weight for the nearest neighbors distance computation. */
            void setWc(double wc)
            {
                wc_ = wc;
            }

            /** \brief Get the cost weight for the nearest neighbors distance computation. */
            double getWc() const 
            {
                return wc_;
            }

            void getPlannerData(base::PlannerData &data) const override;

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            void setup() override;

        protected:
            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation *si)
                  : state(si->allocState()), control(si->allocControl())
                {
                }

                ~Motion() = default;

                /** \brief The state contained by the motion */
                base::State *state{nullptr};

                 /** \brief The cost parameter of the state, contained by the motion */
                double cost{0};
                

                /** \brief The control contained by the motion */
                Control *control{nullptr};

                /** \brief The number of steps the control is applied for */
                unsigned int steps{0};

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief Control sampler */
            DirectedControlSamplerPtr controlSampler_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformation *siC_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{0.05};

            /** \brief The state weight */
            double wx_{1.0};

            /** \brief The cost weight */
            double wc_{0.08};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};

            /** \brief The maximum cost that is used for sampling the cost space using a uniform distribution */
            double cmax_{0};
            /** \brief A flag indicating whether an initial solution has been found. */
            bool isInit{false};
            /** \brief A variable keeping track of the iteration count */
            long iter{0};
        };
    }
}

#endif
