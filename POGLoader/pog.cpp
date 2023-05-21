/** pog.cpp

   \copyright Copyright © CLEARSY 2022
   \license This file is part of POGLoader.

   POGLoader is free software: you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

    POGLoader is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with POGLoader. If not, see <https://www.gnu.org/licenses/>.
*/
#include <iostream>

#include "tinyxml2.h"

#include "btypeReader.h"
#include "exprDesc.h"
#include "predDesc.h"
#include "exprReader.h"
#include "predReader.h"
#include "exprWriter.h"
#include "predWriter.h"
#include "gpredReader.h"
#include "substReader.h"

#include "pog.h"


bool isConj(const Pred &p){
    return (p.getTag() == Pred::PKind::Conjunction);
}

pog::Set readSet(const std::vector<BType> &typeInfos, const tinyxml2::XMLElement *dom){
    std::vector<TypedVar> vec;
    const tinyxml2::XMLElement *elt {dom->FirstChildElement("Enumerated_Values")->FirstChildElement("Id")};
    while(nullptr != elt){
        vec.push_back(Xml::VarNameFromId(elt,typeInfos));
        elt = elt->NextSiblingElement("Id");
    }
    const tinyxml2::XMLElement *id {dom->FirstChildElement("Id")};
    if(nullptr == id)
        throw pog::PogException ("Missing 'Id' element.");
    return pog::Set(Xml::VarNameFromId(id,typeInfos), vec);
}

pog::Pog pog::read(const tinyxml2::XMLDocument *pog){
    Pog res;
    const tinyxml2::XMLElement *root {pog->FirstChildElement("Proof_Obligations")};
    if(nullptr == root)
        throw PogException("Proof_Obligations element expected.");
    // TypeInfos
    std::vector<BType> typeInfos;
    Xml::readTypeInfos(root->FirstChildElement("TypeInfos"),typeInfos);
    // Defines
    for(const tinyxml2::XMLElement *e = root->FirstChildElement("Define");
            nullptr != e;
            e = e->NextSiblingElement("Define"))
    {
        if(nullptr == e->Attribute("name"))
            throw PogException("Attribute name expected.");

        uint64_t hash;
        if(nullptr == e->Attribute("hash"))
            throw PogException("Missing hash attribute.");
        if(tinyxml2::XML_SUCCESS != e->QueryUnsigned64Attribute("hash", &hash))
            throw PogException("Cannot recover hash attribute value.");

        auto def = Define(e->Attribute("name"), hash);
        for(const tinyxml2::XMLElement *ch {e->FirstChildElement()};
                nullptr != ch;
                ch = ch->NextSiblingElement())
        {
            if(0 == strcmp(ch->Name(), "Set")) {
                Set s = readSet(typeInfos,ch);
                def.gsets.push_back(s);
            } else {
                Pred p = Xml::readPredicate(ch,typeInfos);
                assert(!isConj(p));
                def.ghyps.push_back(std::move(p));
            }
        }
        res.defines.push_back(std::move(def));
    }
    // Proof_Obligation
    for(const tinyxml2::XMLElement *po {root->FirstChildElement("Proof_Obligation")};
            nullptr != po;
            po = po->NextSiblingElement("Proof_Obligation"))
    {
        // goalHash
        uint64_t goalHash;
        if(nullptr == po->Attribute("goalHash"))
            throw PogException("Missing goalHash attribute.");
        if(tinyxml2::XML_SUCCESS != po->QueryUnsigned64Attribute("hash", &goalHash))
            throw PogException("Cannot recover hash attribute value.");
        // Tag
        const tinyxml2::XMLElement *e {po->FirstChildElement("Tag")};
        if(nullptr == e)
            throw PogException("Tag element expected.");
        std::string tag {e->GetText()};
        // Definitions
        std::vector<std::string> definitions;
        for(const tinyxml2::XMLElement *e {po->FirstChildElement("Definition")};
                nullptr != e;
                e = e->NextSiblingElement("Definition"))
        {
            definitions.push_back(e->Attribute("Name"));
        }
        // Hypothesis
        std::vector<Pred> hyps;
        for(const tinyxml2::XMLElement *e {po->FirstChildElement("Hypothesis")};
                nullptr != e;
                e = e->NextSiblingElement("Hypothesis"))
        {
            Pred p = Xml::readPredicate(e->FirstChildElement(),typeInfos);
            assert(!isConj(p));
            hyps.push_back(std::move(p));
        }
        // Local Hypotheses
        std::vector<Pred> localHyps;
        for(const tinyxml2::XMLElement *e {po->FirstChildElement("Local_Hyp")};
                nullptr != e;
                e = e->NextSiblingElement("Local_Hyp"))
        {
            Pred p = Xml::readPredicate(e->FirstChildElement(),typeInfos);
            assert(!isConj(p));
            localHyps.push_back(std::move(p));
        }
        // Simple Goal
        std::vector<PO> simpleGoals;
        for(const tinyxml2::XMLElement *e {po->FirstChildElement("Simple_Goal")};
                nullptr != e;
                e = e->NextSiblingElement("Simple_Goal"))
        {
            // Tag
            const tinyxml2::XMLElement *tag = e->FirstChildElement("Tag");
            std::string _tag = tag->GetText();
            // Ref Hyps
            std::vector<int> _localHypRefs;
            for(const tinyxml2::XMLElement *ch = e->FirstChildElement("Ref_Hyp");
                    nullptr != ch;
                    ch = ch->NextSiblingElement("Ref_Hyp"))
            {
                _localHypRefs.push_back(ch->IntAttribute("num"));
            }
            // Goal
            const tinyxml2::XMLElement *goal {e->FirstChildElement("Goal")};
            Pred _goal = Xml::readPredicate(goal->FirstChildElement(),typeInfos);
            simpleGoals.push_back({_tag,_localHypRefs,std::move(_goal)});
        }
        res.pos.push_back(POGroup(tag,goalHash,definitions,std::move(hyps),std::move(localHyps),std::move(simpleGoals)));
    }
    return res;
}
