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

    std::vector<double> scores;
    std::vector<Pylon::DataProcessing::SPointF2D> positions_px;
    std::vector<double> vScalingFactors;
    std::vector<double> hScalingFactors;
    std::vector<Pylon::DataProcessing::SPointF2D> positions_m;

    bool hasError;                  // If something doesn't work as expected
                                    // while processing data, this is set to true.
    Pylon::String_t errorMessage;   // Contains an error message if
                                    // hasError has been set to true.

    void fromVariantContainer(const Pylon::DataProcessing::CVariantContainer& variantContainer)
    {
        // Scores
        auto posScores = variantContainer.find("Scores");
        if (posScores != variantContainer.end())
        {
            const Pylon::DataProcessing::CVariant& value = posScores->second;
            if (value.HasError() == false)
            {
                for(size_t i = 0; i < value.GetNumArrayValues(); ++i)
                {
                    const Pylon::DataProcessing::CVariant scoresValue = value.GetArrayValue(i);
                    if (scoresValue.HasError() == false)
                    {
                        scores.push_back(scoresValue.ToDouble());
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

        // Positions_px
        auto posPositions_px = variantContainer.find("Positions_px");
        if (posPositions_px != variantContainer.end())
        {
            const Pylon::DataProcessing::CVariant& value = posPositions_px->second;
            if (value.HasError() == false)
            {
                for(size_t i = 0; i < value.GetNumArrayValues(); ++i)
                {
                    const Pylon::DataProcessing::CVariant positions_pxValue = value.GetArrayValue(i);
                    if (positions_pxValue.HasError() == false)
                    {
                        positions_px.push_back(positions_pxValue.ToPointF2D());
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

        // VScalingFactors
        auto posVScalingFactors = variantContainer.find("VScalingFactors");
        if (posVScalingFactors != variantContainer.end())
        {
            const Pylon::DataProcessing::CVariant& value = posVScalingFactors->second;
            if (value.HasError() == false)
            {
                for(size_t i = 0; i < value.GetNumArrayValues(); ++i)
                {
                    const Pylon::DataProcessing::CVariant vScalingFactorsValue = value.GetArrayValue(i);
                    if (vScalingFactorsValue.HasError() == false)
                    {
                        vScalingFactors.push_back(vScalingFactorsValue.ToDouble());
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

        // HScalingFactors
        auto posHScalingFactors = variantContainer.find("HScalingFactors");
        if (posHScalingFactors != variantContainer.end())
        {
            const Pylon::DataProcessing::CVariant& value = posHScalingFactors->second;
            if (value.HasError() == false)
            {
                for(size_t i = 0; i < value.GetNumArrayValues(); ++i)
                {
                    const Pylon::DataProcessing::CVariant hScalingFactorsValue = value.GetArrayValue(i);
                    if (hScalingFactorsValue.HasError() == false)
                    {
                        hScalingFactors.push_back(hScalingFactorsValue.ToDouble());
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

        // Positions_m
        auto posPositions_m = variantContainer.find("Positions_m");
        if (posPositions_m != variantContainer.end())
        {
            const Pylon::DataProcessing::CVariant& value = posPositions_m->second;
            if (value.HasError() == false)
            {
                for(size_t i = 0; i < value.GetNumArrayValues(); ++i)
                {
                    const Pylon::DataProcessing::CVariant positions_mValue = value.GetArrayValue(i);
                    if (positions_mValue.HasError() == false)
                    {
                        positions_m.push_back(positions_mValue.ToPointF2D());
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