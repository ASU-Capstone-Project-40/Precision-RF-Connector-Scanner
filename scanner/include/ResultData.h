// Include files to use the pylon API.
#include <pylon/PylonIncludes.h>
// Extend the pylon API for using pylon data processing.
#include <pylondataprocessing/PylonDataProcessingIncludes.h>
// The sample uses the std::vector.
#include <vector>

// Declare a data class for one set of output data values.
class ResultData
{
public:
    ResultData()
        : hasError(false)
    {
    }

    std::vector<double> fixed_score;
    std::vector<Pylon::DataProcessing::SPointF2D> fixed_position;
    std::vector<double> mobile_score;
    std::vector<Pylon::DataProcessing::SPointF2D> mobile_position;

    bool hasError;                  // If something doesn't work as expected
                                    // while processing data, this is set to true.
    Pylon::String_t errorMessage;   // Contains an error message if
                                    // hasError has been set to true.

    void fromVariantContainer(const Pylon::DataProcessing::CVariantContainer& variantContainer)
    {
        // fixed_score
        auto posfixed_score = variantContainer.find("fixed_score");
        if (posfixed_score != variantContainer.end())
        {
            const Pylon::DataProcessing::CVariant& value = posfixed_score->second;
            if (value.HasError() == false)
            {
                for(size_t i = 0; i < value.GetNumArrayValues(); ++i)
                {
                    const Pylon::DataProcessing::CVariant fixed_scoreValue = value.GetArrayValue(i);
                    if (fixed_scoreValue.HasError() == false)
                    {
                        fixed_score.push_back(fixed_scoreValue.ToDouble());
                    }
                    else
                    {
                        hasError = true;
                        errorMessage = value.GetErrorDescription();
                        break;
                    }
                }
            }
            else
            {
                hasError = true;
                errorMessage = value.GetErrorDescription();
            }
        }

        // fixed_position
        auto posfixed_position = variantContainer.find("fixed_position");
        if (posfixed_position != variantContainer.end())
        {
            const Pylon::DataProcessing::CVariant& value = posfixed_position->second;
            if (value.HasError() == false)
            {
                for(size_t i = 0; i < value.GetNumArrayValues(); ++i)
                {
                    const Pylon::DataProcessing::CVariant fixed_positionValue = value.GetArrayValue(i);
                    if (fixed_positionValue.HasError() == false)
                    {
                        fixed_position.push_back(fixed_positionValue.ToPointF2D());
                    }
                    else
                    {
                        hasError = true;
                        errorMessage = value.GetErrorDescription();
                        break;
                    }
                }
            }
            else
            {
                hasError = true;
                errorMessage = value.GetErrorDescription();
            }
        }

        // mobile_score
        auto posmobile_score = variantContainer.find("mobile_score");
        if (posmobile_score != variantContainer.end())
        {
            const Pylon::DataProcessing::CVariant& value = posmobile_score->second;
            if (value.HasError() == false)
            {
                for(size_t i = 0; i < value.GetNumArrayValues(); ++i)
                {
                    const Pylon::DataProcessing::CVariant mobile_scoreValue = value.GetArrayValue(i);
                    if (mobile_scoreValue.HasError() == false)
                    {
                        mobile_score.push_back(mobile_scoreValue.ToDouble());
                    }
                    else
                    {
                        hasError = true;
                        errorMessage = value.GetErrorDescription();
                        break;
                    }
                }
            }
            else
            {
                hasError = true;
                errorMessage = value.GetErrorDescription();
            }
        }

        // mobile_position
        auto posmobile_position = variantContainer.find("mobile_position");
        if (posmobile_position != variantContainer.end())
        {
            const Pylon::DataProcessing::CVariant& value = posmobile_position->second;
            if (value.HasError() == false)
            {
                for(size_t i = 0; i < value.GetNumArrayValues(); ++i)
                {
                    const Pylon::DataProcessing::CVariant mobile_positionValue = value.GetArrayValue(i);
                    if (mobile_positionValue.HasError() == false)
                    {
                        mobile_position.push_back(mobile_positionValue.ToPointF2D());
                    }
                    else
                    {
                        hasError = true;
                        errorMessage = value.GetErrorDescription();
                        break;
                    }
                }
            }
            else
            {
                hasError = true;
                errorMessage = value.GetErrorDescription();
            }
        }

    }
};