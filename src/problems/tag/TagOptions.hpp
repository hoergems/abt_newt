/** @file TagOptions.hpp
 *
 * Defines the TagOptions class, which specifies the configuration settings available for the
 * Tag problem.
 */
#ifndef TAGOPTIONS_HPP_
#define TAGOPTIONS_HPP_

#include <string>                       // for string

#include "problems/shared/SharedOptions.hpp"

namespace tag {
/** A class defining the configuration settings for the Tag problem. */
struct TagOptions : public shared::SharedOptions {
    TagOptions() = default;
    virtual ~TagOptions() = default;

    /* -------- Settings specific to the Tag POMDP -------- */
    /** Path to the map file (relative to SharedOptions::baseConfigPath) */
    std::string mapPath = "";
    /** Cost per move. */
    double moveCost = 0.0;
    /** Reward for tagging. */
    double tagReward = 0.0;
    /** Penalty for a failed tag attempt. */
    double failedTagPenalty = 0.0;
    /** Probability the opponent will stay in place. */
    double opponentStayProbability = 0.0;
    /** Path to vrep scene tag.ttt */
    std::string vrepScenePath = "";

    /** Constructs an OptionParser instance that will parse configuration settings for the Tag
     * problem into an instance of TagOptions.
     */
    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {
        std::unique_ptr<options::OptionParser> parser = SharedOptions::makeParser(simulating,
                EXPAND_AND_QUOTE(ROOT_PATH) "/problems/tag");
        addTagOptions(parser.get());
        return std::move(parser);
    }

    /** Adds the core configuration settings for the Tag problem to the given parser. */
    static void addTagOptions(options::OptionParser *parser) {
        parser->addOption<std::string>("problem", "mapPath", &TagOptions::mapPath);
        parser->addValueArg<std::string>("problem", "mapPath", &TagOptions::mapPath,
                "", "map", "the path to the map file (relative to the base config path)", "path");

        parser->addOption<double>("problem", "moveCost", &TagOptions::moveCost);
        parser->addOption<double>("problem", "tagReward", &TagOptions::tagReward);
        parser->addOption<double>("problem", "failedTagPenalty", &TagOptions::failedTagPenalty);
        parser->addOption<double>("problem", "opponentStayProbability",
                &TagOptions::opponentStayProbability);
        parser->addOptionWithDefault<std::string>("ros", "vrepScenePath",
                &TagOptions::vrepScenePath, "");
    }
};
} /* namespace tag */

#endif /* TAGOPTIONS_HPP_ */
