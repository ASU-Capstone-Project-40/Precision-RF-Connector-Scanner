// Include files to use the pylon API.
#include <pylon/PylonIncludes.h>

// Extend the pylon API for using pylon data processing.
#include <pylondataprocessing/PylonDataProcessingIncludes.h>

// The sample uses the std::list.
#include <list>


#include "ResultData.h"
#include "OutputObserver.h"

// Namespaces for using pylon objects
using namespace Pylon;
using namespace Pylon::DataProcessing;

// Namespace for using cout
using namespace std;

// Number of processing iterations
static const uint32_t c_iterations = 100;

int main(int /*argc*/, char* /*argv*/[])
{
    // The exit code of the sample application.
    int exitCode = 0;

    // Before using any pylon methods, the pylon runtime must be initialized.
    PylonInitialize();

    try
    {
        // This object is used for collecting the output data.
        // If placed on the stack, it must be created before the recipe
        // so that it is destroyed after the recipe.
        RecipeOutputObserver resultCollector;

        // Create a recipe object representing a recipe file created using
        // the pylon Viewer Workbench.
        CRecipe recipe;

        // Load the recipe file.
        recipe.Load(PYLON_RECIPE);

        // Now we allocate all resources we need. This includes the camera device if used in the recipe.
        recipe.PreAllocateResources();

        // This is where the output goes.
        recipe.RegisterAllOutputsObserver(&resultCollector, RegistrationMode_Append);

        // Start the processing.
        recipe.Start();

        for (;;)
        {
            if (resultCollector.GetWaitObject().Wait(5000))
            {
                ResultData result;
                resultCollector.GetResultData(result);
                if (!result.hasError)
                {
                    if (!result.scores.empty()) {
                        cout << "resultData collected from recipe image_recon" << endl;
                        cout << "Score: " << result.scores[0] << endl;
                        cout << "Position: " << result.positions_px[0].X << ", " << result.positions_px[0].Y << endl;
                    }
                    else {
                        cout << "No object detected." << endl;
                    }
                }
                else
                {
                    cout << "An error occurred during processing recipe image_recon: " << result.errorMessage << endl;
                }
            }
            else
            {
                throw RUNTIME_EXCEPTION("Result timeout");
            }
        }
        // Stop the processing.
        recipe.Stop();

    }
    catch (const GenericException& e)
    {
        // Error handling
        cerr << "An exception occurred." << endl << e.GetDescription() << endl;
        exitCode = 1;
    }

    // Releases all pylon resources.
    PylonTerminate();

    return exitCode;
}