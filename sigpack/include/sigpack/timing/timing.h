
/*------------------------------------------------------------------------------
* Copyright (C) 2020-2021 LEIDOS.
*
* Licensed under the Apache License, Version 2.0 (the "License"); you may not
* use this file except in compliance with the License. You may obtain a copy of
* the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
* License for the specific language governing permissions and limitations under
* the License.

------------------------------------------------------------------------------*/

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.
#ifndef SP_TIMING_H
#define SP_TIMING_H
namespace sp
{
    ///
    /// @defgroup timing Timing
    /// \brief Timing functions.
    /// @{

    ///
    /// \brief A delay class.
    ///
    /// Implements different timing related functions such as delay
    ///
    template <class T1>
    class Delay
    {
        private:
            arma::uword D;        ///< The delay value
            arma::uword cur_p;    ///< Pointer to current sample in buffer
            arma::Col<T1> buf;    ///< Signal buffer
        public:
            ////////////////////////////////////////////////////////////////////////////////////////////
            /// \brief Constructor.
            ////////////////////////////////////////////////////////////////////////////////////////////
            Delay()
            {
                cur_p = 0;
                D = 0;
            }

            ////////////////////////////////////////////////////////////////////////////////////////////
            /// \brief Constructor with delay input.
            /// @param _D delay
            ////////////////////////////////////////////////////////////////////////////////////////////
            Delay(const arma::uword _D)
            {
                set_delay(_D);
                clear();
            }

            ////////////////////////////////////////////////////////////////////////////////////////////
            /// \brief Destructor.
            ////////////////////////////////////////////////////////////////////////////////////////////
            ~Delay() {}

            ////////////////////////////////////////////////////////////////////////////////////////////
            ///  \brief Clears internal state.
            ////////////////////////////////////////////////////////////////////////////////////////////
            void clear(void)
            {
                buf.zeros();
                cur_p = 0;
            }

            ////////////////////////////////////////////////////////////////////////////////////////////
            /// \brief Sets delay.
            /// @param _D delay
            ////////////////////////////////////////////////////////////////////////////////////////////
            void set_delay(const arma::uword _D)
            {
                D = _D+1;
                buf.set_size(D);
            }

            ////////////////////////////////////////////////////////////////////////////////////////////
            /// \brief A delay operator.
            /// @param in sample input
            ////////////////////////////////////////////////////////////////////////////////////////////
            T1 operator()(const T1& in)
            {
                buf[cur_p] = in;                    // Insert new sample
                // Move insertion point
                if (cur_p == 0)
                    cur_p = D-1;
                else
                    cur_p--;

                return buf[cur_p];
            }

            ////////////////////////////////////////////////////////////////////////////////////////////
            /// \brief A delay operator (vector version).
            /// @param in vector input
            ////////////////////////////////////////////////////////////////////////////////////////////
            arma::Col<T1> delay(const arma::Col<T1>& in)
            {
                arma::uword sz = in.size();
                arma::Col<T1> out(sz);
                for(arma::uword n=0; n<sz; n++)
                    out[n] = this->operator()(in[n]);
                return out;
            }
    };
    /// @}

} // end namespace
#endif
