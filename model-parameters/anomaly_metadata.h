/* Generated by Edge Impulse
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _EI_CLASSIFIER_ANOMALY_METADATA_H_
#define _EI_CLASSIFIER_ANOMALY_METADATA_H_

#include "edge-impulse-sdk/classifier/ei_model_types.h"

const uint16_t ei_classifier_anom_axes[] = { 0, 13, 26 };

// (before - mean) / scale
const float ei_classifier_anom_scale[3] = { 4.778433430393973, 2.7600960618663057, 1.5945312435603232 };
const float ei_classifier_anom_mean[3] = { 4.369189092317084, 2.4612239079604414, 2.5223747687911855 };
const ei_classifier_anom_cluster_t ei_classifier_anom_clusters[32] = {
	{ ( float[3] ) { -0.713545024394989, -0.3662114143371582, -0.45544561743736267 }, 0.21545905996919165 },
	{ ( float[3] ) { 0.006694763898849487, 3.6990199089050293, -0.413569837808609 }, 0.6053889374342187 },
	{ ( float[3] ) { -0.6819738745689392, -0.3386862576007843, 1.5125148296356201 }, 0.325333778843825 },
	{ ( float[3] ) { -0.9100226163864136, -0.8859167695045471, -0.4518221318721771 }, 0.040132914204607704 },
	{ ( float[3] ) { 1.1816540956497192, -0.2484085112810135, -0.508625864982605 }, 0.40265791053181194 },
	{ ( float[3] ) { 0.9852532744407654, -0.04817153885960579, -0.7675943970680237 }, 0.36439507009954 },
	{ ( float[3] ) { -0.5257946252822876, 0.3770110309123993, -0.05470715090632439 }, 0.34216274196032853 },
	{ ( float[3] ) { 1.9336779117584229, 0.0707564428448677, 1.2861605882644653 }, 0.6475493751492741 },
	{ ( float[3] ) { 1.7868112325668335, 0.2692870795726776, 0.09685646742582321 }, 0.4915811755678379 },
	{ ( float[3] ) { -0.853087842464447, -0.26231300830841064, -1.5570979118347168 }, 0.06593700481396238 },
	{ ( float[3] ) { -0.4028180241584778, 0.7304807305335999, 0.2016018033027649 }, 0.7524225369948614 },
	{ ( float[3] ) { 1.6406018733978271, 0.6987727284431458, 0.20899340510368347 }, 0.4435801207574787 },
	{ ( float[3] ) { 1.3385496139526367, -0.3807240128517151, 0.29214444756507874 }, 0.4558325753485982 },
	{ ( float[3] ) { -0.653103232383728, -0.180096834897995, -0.46766379475593567 }, 0.2793879298577588 },
	{ ( float[3] ) { 1.2930078506469727, 0.20768152177333832, -0.7782511711120605 }, 0.44612826344110673 },
	{ ( float[3] ) { -0.4317224323749542, -0.3810184597969055, 2.5322482585906982 }, 0.731755208301942 },
	{ ( float[3] ) { 1.3571709394454956, -0.21993449330329895, -0.8451143503189087 }, 0.4742460291609575 },
	{ ( float[3] ) { 1.9408034086227417, 0.038564518094062805, 0.5897795557975769 }, 0.3631808861937634 },
	{ ( float[3] ) { -0.7469649910926819, -0.5735572576522827, 1.0856423377990723 }, 0.5049470375130225 },
	{ ( float[3] ) { 1.4735397100448608, 0.5791605114936829, -0.4027330279350281 }, 0.44187473428149304 },
	{ ( float[3] ) { -0.5197398066520691, 0.07302822172641754, 1.3893455266952515 }, 0.4469945098222016 },
	{ ( float[3] ) { -0.2195717692375183, 3.4199013710021973, -0.7546936273574829 }, 0.556353427479358 },
	{ ( float[3] ) { -0.3694555461406708, -0.17739041149616241, 3.03556752204895 }, 0.6410838835882963 },
	{ ( float[3] ) { -0.5360965728759766, -0.8846145272254944, -1.5337234735488892 }, 0.008649659281079369 },
	{ ( float[3] ) { 0.9504920244216919, -0.08206669986248016, 0.48554128408432007 }, 0.41066597143441685 },
	{ ( float[3] ) { -0.26966333389282227, 1.6483957767486572, -0.3034801185131073 }, 0.6246270595585354 },
	{ ( float[3] ) { 1.03476083278656, 3.3015024662017822, 1.0422791242599487 }, 0.3332048918068076 },
	{ ( float[3] ) { -0.6323502063751221, -0.07798077166080475, 1.8054287433624268 }, 0.38925864243671143 },
	{ ( float[3] ) { -0.5033231377601624, -0.36682334542274475, 2.0517020225524902 }, 0.34841948955169083 },
	{ ( float[3] ) { 1.032531499862671, 0.0026600947603583336, -0.04568548873066902 }, 0.36966465630512085 },
	{ ( float[3] ) { 0.8068602085113525, 2.740093469619751, 0.5661335587501526 }, 0.5510816456685511 },
	{ ( float[3] ) { -0.21534320712089539, 1.0939421653747559, 2.4646966457366943 }, 0.35572841620935675 },
};

#endif // _EI_CLASSIFIER_ANOMALY_METADATA_H_