/**
* This file is part of FVO.
* Viewer: response for camera poses , point cloud and images show
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 28th Jan, 2018
*/

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "fvo/Viewer.h"

namespace fvo{

    Viewer::Viewer( bool startViewer )
    {
        if( startViewer ) {
            mbRunning = true;
            mViewerThread = thread(&Viewer::run, this );
        }
        mBiasLogLabels.clear();
        mBiasLogLabels.push_back("bg_x");
        mBiasLogLabels.push_back("bg_y");
        mBiasLogLabels.push_back("bg_z");
        mBiasLogLabels.push_back("ba_x");
        mBiasLogLabels.push_back("ba_y");
        mBiasLogLabels.push_back("ba_z");
        mBiasLog.SetLabels(mBiasLogLabels);
    }

    Viewer::~Viewer()
    {

    }

    void Viewer::run() {
        int w = G::imageWidth;
        int h = G::imageHeight;
        pangolin::CreateWindowAndBind("FVO-Viewer", G::viewerWidth, G::viewerHeight );

        Vector3d up(0,-1,0);

        // 3D viewer
        pangolin::OpenGlRenderState visual3D_camera(
                pangolin::ProjectionMatrix(1024, 768, 500, 500, w / 2, h / 2, 0.1, 1000),
                pangolin::ModelViewLookAt(0, 0, -0.5, 0, 0, 0, up[0], up[1], up[2])
        );
        pangolin::View &visual3d_dispaly = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(G::UI_WIDTH), 1.0, -w/(float)h)
                .SetHandler( new pangolin::Handler3D(visual3D_camera));

        pangolin::CreatePanel("UI")
                .SetBounds(0.6, 1.0, 0.0, pangolin::Attach::Pix(G::UI_WIDTH));

        // Show the curve of IMU bias
        pangolin::Plotter plotter_biascurve(&mBiasLog, 0.0f, 40.0f * 20, -0.4f, 0.4f, 1.0f / 20, 0.1f);
        plotter_biascurve.Track("$i");
        plotter_biascurve.SetAspect(w / (float) h);
        pangolin::CreateDisplay()
                .SetBounds(0.3, 0.6, 0.0, pangolin::Attach::Pix(G::UI_WIDTH))
                .AddDisplay(plotter_biascurve);

        pangolin::CreatePanel("ui")
                .SetBounds(0.6, 1.0, 0.0, pangolin::Attach::Pix(G::UI_WIDTH));

        pangolin::Var<int> nTrackFeats("ui.TrackInliers", 0 );
        pangolin::Var<int> nTrackState("ui.TrackState", 0 );

        while( !pangolin::ShouldQuit() && mbRunning ) {
            // Todo
            // judge the new frame update?



            // clear entire screen
            //glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
            // show frames
            // Activate efeiciently by object
            visual3d_dispaly.Activate( visual3D_camera );
            DrawOrigin();

            // Todo
            // draw all items


            // Swap frames and process events
            pangolin::FinishFrame();
            usleep( 3*10000 );

            // Todo
            LOG(WARNING) << "Todo: Viewer::run()" <<  std::endl;

        }
        mbRunning = false;
    }


    // draw the original
    void Viewer::DrawOrigin() {
        // x axis
        glColor3d(1, 0, 0);     // red is x
        glLineWidth(4);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(1, 0, 0);
        glEnd();

        // y axis
        glColor3d(0, 1, 0);     // green is y
        glLineWidth(4);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 1, 0);
        glEnd();

        // z axis
        glColor3d(0, 0, 1);     // blue is z
        glLineWidth(4);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 1);
        glEnd();
    }

} // end of namespace