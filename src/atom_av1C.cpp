/*
 * The contents of this file are subject to the Mozilla Public
 * License Version 1.1 (the "License"); you may not use this file
 * except in compliance with the License. You may obtain a copy of
 * the License at http://www.mozilla.org/MPL/
 *
 * Software distributed under the License is distributed on an "AS
 * IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
 * implied. See the License for the specific language governing
 * rights and limitations under the License.
 *
 * The Original Code is MPEG4IP.
 *
 * The Initial Developer of the Original Code is Cisco Systems Inc.
 * Portions created by Cisco Systems Inc. are
 * Copyright (C) Cisco Systems Inc. 2004.  All Rights Reserved.
 *
 * Contributor(s):
 *      Bill May wmay@cisco.com
 */

#include "src/impl.h"

namespace mp4v2 {
namespace impl {

///////////////////////////////////////////////////////////////////////////////

MP4Av1CAtom::MP4Av1CAtom(MP4File &file)
        : MP4Atom(file, "av1C")
{
    AddProperty( new MP4BitfieldProperty(*this, "marker", 1)); /* 0 */
    AddProperty( new MP4BitfieldProperty(*this, "version", 7)); /* 1 */
    AddProperty( new MP4BitfieldProperty(*this, "seq_profile", 3)); /* 2 */
    AddProperty( new MP4BitfieldProperty(*this, "seq_level_idx_0", 5)); /* 3 */
    AddProperty( new MP4BitfieldProperty(*this, "seq_tier_0", 1)); /* 4 */
    AddProperty( new MP4BitfieldProperty(*this, "high_bitdepth", 1)); /* 5 */
    AddProperty( new MP4BitfieldProperty(*this, "twelve_bit", 1)); /* 6 */
    AddProperty( new MP4BitfieldProperty(*this, "monochrome", 1)); /* 7 */
    AddProperty( new MP4BitfieldProperty(*this, "chroma_subsampling_x", 1)); /* 8 */
    AddProperty( new MP4BitfieldProperty(*this, "chroma_subsampling_y", 1)); /* 9 */
    AddProperty( new MP4BitfieldProperty(*this, "chroma_sample_position", 2)); /* 10 */
    AddProperty( new MP4BitfieldProperty(*this, "reserved ", 3)); /* 11 */
    AddProperty( new MP4BitfieldProperty(*this, "initial_presentation_delay_present", 1)); /* 12 */
    AddProperty( new MP4BitfieldProperty(*this, "initial_presentation_delay_minus_one_or_reserved", 4)); /* 13 */
    AddProperty( new MP4BytesProperty(*this, "configOBUs", 0)); /* 14 */
}


void MP4Av1CAtom::Generate()
{
    MP4Atom::Generate();
    ((MP4BitfieldProperty*)m_pProperties[0])->SetValue(1);
    ((MP4BitfieldProperty*)m_pProperties[1])->SetValue(1);
    ((MP4BitfieldProperty*)m_pProperties[2])->SetValue(0);
    ((MP4BitfieldProperty*)m_pProperties[3])->SetValue(0);
    ((MP4BitfieldProperty*)m_pProperties[4])->SetValue(0);
    ((MP4BitfieldProperty*)m_pProperties[5])->SetValue(0);
    ((MP4BitfieldProperty*)m_pProperties[6])->SetValue(0);
    ((MP4BitfieldProperty*)m_pProperties[7])->SetValue(0);
    ((MP4BitfieldProperty*)m_pProperties[8])->SetValue(0);
    ((MP4BitfieldProperty*)m_pProperties[9])->SetValue(0);
    ((MP4BitfieldProperty*)m_pProperties[10])->SetValue(0);
    ((MP4BitfieldProperty*)m_pProperties[11])->SetValue(0);
    ((MP4BitfieldProperty*)m_pProperties[12])->SetValue(0);
    ((MP4BitfieldProperty*)m_pProperties[13])->SetValue(0);
}
///////////////////////////////////////////////////////////////////////////////

}
} // namespace mp4v2::impl
