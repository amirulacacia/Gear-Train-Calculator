#include "Application.h"
#include "imgui.h"
#include "implot.h"
#include "implot_internal.h"

namespace MyApp
{

    void RenderUI()
    {
        //------------------------------------------------------
        // Tools code
        //------------------------------------------------------
        
        //------------------------------------------------------
        // DockSpace code
        //------------------------------------------------------
        static bool opt_fullscreen = true;
        static bool opt_padding = false;
        static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_None;

        // We are using the ImGuiWindowFlags_NoDocking flag to make the parent window not dockable into,
        // because it would be confusing to have two docking targets within each others.
        ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
        if (opt_fullscreen)
        {
            const ImGuiViewport* viewport = ImGui::GetMainViewport();
            ImGui::SetNextWindowPos(viewport->WorkPos);
            ImGui::SetNextWindowSize(viewport->WorkSize);
            ImGui::SetNextWindowViewport(viewport->ID);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
            window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
            window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
        }
        else
        {
            dockspace_flags &= ~ImGuiDockNodeFlags_PassthruCentralNode;
        }

        // When using ImGuiDockNodeFlags_PassthruCentralNode, DockSpace() will render our background
        // and handle the pass-thru hole, so we ask Begin() to not render a background.
        if (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode)
            window_flags |= ImGuiWindowFlags_NoBackground;

        // Important: note that we proceed even if Begin() returns false (aka window is collapsed).
        // This is because we want to keep our DockSpace() active. If a DockSpace() is inactive,
        // all active windows docked into it will lose their parent and become undocked.
        // We cannot preserve the docking relationship between an active window and an inactive docking, otherwise
        // any change of dockspace/settings would lead to windows being stuck in limbo and never being visible.
        if (!opt_padding)
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
        ImGui::Begin("Gear Ratio Calculator", nullptr, window_flags);
        if (!opt_padding)
            ImGui::PopStyleVar();

        if (opt_fullscreen)
            ImGui::PopStyleVar(2);

        // Submit the DockSpace
        ImGuiIO& io = ImGui::GetIO();
        if (io.ConfigFlags & ImGuiConfigFlags_DockingEnable)
        {
            ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
            ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);
        }
        static bool show_app_style_editor = false;
        if (ImGui::BeginMenuBar())
        {
            if (ImGui::BeginMenu("Options"))
            {
                // Disabling fullscreen would allow the window to be moved to the front of other windows,
                // which we can't undo at the moment without finer window depth/z control.
                ImGui::MenuItem("Fullscreen", NULL, &opt_fullscreen);
                ImGui::MenuItem("Padding", NULL, &opt_padding);
                ImGui::Separator();

                if (ImGui::MenuItem("Flag: NoSplit", "", (dockspace_flags & ImGuiDockNodeFlags_NoSplit) != 0)) { dockspace_flags ^= ImGuiDockNodeFlags_NoSplit; }
                if (ImGui::MenuItem("Flag: NoResize", "", (dockspace_flags & ImGuiDockNodeFlags_NoResize) != 0)) { dockspace_flags ^= ImGuiDockNodeFlags_NoResize; }
                if (ImGui::MenuItem("Flag: NoDockingInCentralNode", "", (dockspace_flags & ImGuiDockNodeFlags_NoDockingInCentralNode) != 0)) { dockspace_flags ^= ImGuiDockNodeFlags_NoDockingInCentralNode; }
                if (ImGui::MenuItem("Flag: AutoHideTabBar", "", (dockspace_flags & ImGuiDockNodeFlags_AutoHideTabBar) != 0)) { dockspace_flags ^= ImGuiDockNodeFlags_AutoHideTabBar; }
                if (ImGui::MenuItem("Flag: PassthruCentralNode", "", (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode) != 0, opt_fullscreen)) { dockspace_flags ^= ImGuiDockNodeFlags_PassthruCentralNode; }
                ImGui::Separator();
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Style Editor"))
            {
                show_app_style_editor = true;
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("About"))
            {
                ImGui::Text("Gear Ratio Calculator V1.0");
                ImGui::Separator();
                ImGui::Text("By Muhammad Amirul Irfan Bin Mohd Yusmin (1912259).");
                ImGui::Text("Application for Power Train Project.");
                ImGui::Text("Thanks to Dear ImGui and ImPlot for the amazing GUI library.");
                ImGui::EndMenu();
            }
           
            ImGui::EndMenuBar();
        }

        if (show_app_style_editor)
        {
            ImGui::Begin("Style Editor", &show_app_style_editor);
            ImGui::ShowStyleEditor();
            ImGui::End();
        }

        //------------------------------------------------------
        // Gear Ratio Calculator Code
        //------------------------------------------------------
        ImGui::Begin("Gear Ratio Input");

        

        // Drivetrain variables
        static bool selected[3] = { false, false, false };
        const char* drivetrain[3] = { "FWD (Foward Wheel Drive)", "RWD (Rear Wheel Drive)"/*, "AWD (All Wheel Drive)" */};
        const char* drivetrainshort[3] = { "FWD", "RWD"/*, "AWD"*/ };
        static int gearnumber = 5;

        // Vehicle body variables
        static float mass, weight, frontweight, rearweight,
            hlratio = 0.3f, wheelradius = 0.3f, gravity = 9.81f;

        // Vehicle movement variables
        static float rollingresistance = 0.02f, roadadhesion = 0.9f, overallaerodynamic = 0.35f,
            desiredspeed = 50.f, vehiclespeed;

        // Engine variables
        static float maxenginespeedrpm = 8500.f , maxenginespeedrad = 890.1f , drivelineefficiency = 100.f;
        static double PIvalue = 3.141592653589793238;
        static bool torqueoption[3] = { true, false };

        // Torque Equation variables
        static double coefatorque, coefbtorque, coefctorque = 55.24;
        static double coefa = -4.45, coefb = 3.17;
        static int coefapower = -6, coefbpower = -2;

        // Torque coordinate variables
        static double coordx[1000], coordy[1000], coordpower[1000];
        static int numpoints = 100;
        static double torquemax, powermax, torquespeed, powerspeed;

        coefatorque = coefa * pow(10.f, coefapower);
        coefbtorque = coefb * pow(10.f, coefbpower);

        if (ImGui::CollapsingHeader("Drivetrain", ImGuiTreeNodeFlags_DefaultOpen))
        {
            //------------------------------------------------------
            // Drivetrain Input Code
            //------------------------------------------------------
            for (int i = 0; i < 2; i++)
            {
                if (ImGui::Checkbox(drivetrain[i], &selected[i]))
                {
                    switch (i)
                    {
                    case 0:
                        selected[1] = false;
                        //selected[2] = false;
                        break;
                    case 1:
                        selected[0] = false;
                        //selected[2] = false;
                        break;
                    //case 2:
                        //selected[0] = false;
                        //selected[1] = false;
                        //break;
                    }
                }
            }
            ImGui::PushItemWidth(50);
            ImGui::AlignTextToFramePadding();
            ImGui::Text("Number of Gear"); ImGui::SameLine(110); ImGui::Text(":"); ImGui::SameLine();
            ImGui::InputInt("##gearnumber", &gearnumber, NULL, NULL, ImGuiInputTextFlags_None);
            ImGui::PopItemWidth();

        }
        if (ImGui::CollapsingHeader("Vehicle Body", ImGuiTreeNodeFlags_DefaultOpen))
        {
            //------------------------------------------------------
            // Vehicle Body Input Code
            //------------------------------------------------------
            ImGui::PushItemWidth(50);
            ImGui::AlignTextToFramePadding();
            ImGui::Text("Tyre Rolling Radius"); ImGui::SameLine(148); ImGui::Text(":"); ImGui::SameLine();
            ImGui::InputFloat("##wheelradius", &wheelradius, NULL, NULL, "%.2f");
            ImGui::SameLine(); ImGui::Text("m");

            ImGui::Text("Weight Distribution %:");
            ImGui::AlignTextToFramePadding();
            ImGui::TextWrapped("Front"); ImGui::SameLine(50); ImGui::Text(":"); ImGui::SameLine();
            if (ImGui::InputFloat("##frontweight", &frontweight, NULL, NULL, "%1.f"))
            {
                rearweight = 100 - frontweight;
            }
            ImGui::SameLine(); ImGui::Text("%%"); ImGui::SameLine(160);
            ImGui::Text("Rear"); ImGui::SameLine(210); ImGui::Text(":"); ImGui::SameLine();
            if (ImGui::InputFloat("##rearweight", &rearweight, NULL, NULL, "%1.f"))
            {
                frontweight = 100 - rearweight;
            }
            ImGui::SameLine(); ImGui::Text("%%");

            ImGui::AlignTextToFramePadding();
            ImGui::Text("Mass"); ImGui::SameLine(50); ImGui::Text(":"); ImGui::SameLine();
            if (ImGui::InputFloat("##mass", &mass, NULL, NULL, "%.1f"))
            {
                weight = mass * gravity;
            }
            ImGui::SameLine(); ImGui::Text("kg"); ImGui::SameLine(160);
            ImGui::Text("Weight"); ImGui::SameLine(210); ImGui::Text(":"); ImGui::SameLine();
            if (ImGui::InputFloat("##weight", &weight, NULL, NULL, "%.1f"))
            {
                mass = weight / gravity;
            }
            ImGui::SameLine(); ImGui::Text("N");

            ImGui::AlignTextToFramePadding();
            ImGui::Text("Height / Wheelbase Ratio h/l"); ImGui::SameLine(210); ImGui::Text(":"); ImGui::SameLine();
            ImGui::InputFloat("##hlratio", &hlratio, NULL, NULL, "%.2f");
            ImGui::PopItemWidth();
        }
        if (ImGui::CollapsingHeader("Vehicle Movement", ImGuiTreeNodeFlags_DefaultOpen))
        {
            //------------------------------------------------------
            // Vehicle Movement Input Code
            //------------------------------------------------------
            const int lineoffset = 230;
            ImGui::PushItemWidth(50);
            ImGui::AlignTextToFramePadding();
            ImGui::Text("Desired Maximum Vehicle Speed"); ImGui::SameLine(lineoffset); ImGui::Text(":"); ImGui::SameLine();
            ImGui::InputFloat("##maximumspeed", &desiredspeed, NULL, NULL, "%.2f");
            ImGui::SameLine(); ImGui::PopItemWidth();
            ImGui::PushItemWidth(60);

            // unit selection of vehicle speed
            const char* speedunit[2] = { "m/s", "km/h"};
            static int current_i = 0; // Here we store our selection data as an index.
            const char* preview_i = speedunit[current_i];  // Pass in the preview value visible before opening the combo (it could be anything)
            if (ImGui::BeginCombo("##speedunit", preview_i, ImGuiComboFlags_None))
            {
                for (int n = 0; n < IM_ARRAYSIZE(speedunit); n++)
                {
                    const bool is_selected = (current_i == n);
                    if (ImGui::Selectable(speedunit[n], is_selected))
                        current_i = n;

                    // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                    if (is_selected)
                        ImGui::SetItemDefaultFocus();

                }
                ImGui::EndCombo();
            }
            ImGui::PopItemWidth();
            switch (current_i)
            {
            case 0:
                vehiclespeed = desiredspeed;
                break;
            case 1:
                vehiclespeed = desiredspeed * 1000. / (60. * 60.);
                break;
            }
            //ImGui::Text("vehicle speed: %f m/s", vehiclespeed);   // uncomment to check the speed of vehicle

            ImGui::PushItemWidth(50);
            ImGui::AlignTextToFramePadding();
            ImGui::Text("Road Adhesion Coefficient"); ImGui::SameLine(lineoffset); ImGui::Text(":"); ImGui::SameLine();
            ImGui::InputFloat("##roadadhesion", &roadadhesion, NULL, NULL, "%.2f");

            ImGui::AlignTextToFramePadding();
            ImGui::Text("Rolling Resistance Coefficient"); ImGui::SameLine(lineoffset); ImGui::Text(":"); ImGui::SameLine();
            ImGui::InputFloat("##rollingresistance", &rollingresistance, NULL, NULL, "%.2f");

            ImGui::AlignTextToFramePadding();
            ImGui::Text("Overall Aerodynamic Coefficient"); ImGui::SameLine(lineoffset); ImGui::Text(":"); ImGui::SameLine();
            ImGui::InputFloat("##overallaerodynamic", &overallaerodynamic, NULL, NULL, "%.2f");
            ImGui::PopItemWidth();

        }
        if (ImGui::CollapsingHeader("Engine", ImGuiTreeNodeFlags_DefaultOpen))
        {
            //------------------------------------------------------
            // Engine Input Code
            //------------------------------------------------------
            const int offsetlineengine = 150;
            ImGui::PushItemWidth(50);
            ImGui::AlignTextToFramePadding();
            ImGui::Text("Maximum Engine Speed"); ImGui::SameLine(offsetlineengine); ImGui::Text(":"); ImGui::SameLine();
            if (ImGui::InputFloat("##maxenginespeedrpm", &maxenginespeedrpm, NULL, NULL, "%1.f"))
            {
                maxenginespeedrad = maxenginespeedrpm * 2 * PIvalue / 60;
            }
            ImGui::SameLine(); ImGui::Text("rpm"); ImGui::SameLine();
            if (ImGui::InputFloat("##maxenginespeedrad", &maxenginespeedrad, NULL, NULL, "%.1f"))
            {
                maxenginespeedrpm = maxenginespeedrad * 60 / 2 / PIvalue;
            }
            ImGui::SameLine(); ImGui::Text("rad/s");

            ImGui::AlignTextToFramePadding();
            ImGui::Text("Driveline Efficiency"); ImGui::SameLine(offsetlineengine); ImGui::Text(":"); ImGui::SameLine();
            ImGui::InputFloat("##drivelineefficiency", &drivelineefficiency, NULL, NULL, "%.1f");
            ImGui::SameLine(); ImGui::Text("%%");
            ImGui::PopItemWidth();

            ImGui::AlignTextToFramePadding();
            ImGui::Text("Torque:");
            /*
            ImGui::AlignTextToFramePadding();
            ImGui::Text("Torque Equation:");

            ImGui::PushItemWidth(50);
            ImGui::Text("aw^2 + bw + c , where w is in rpm");
            ImGui::AlignTextToFramePadding();
            ImGui::Text("a"); ImGui::SameLine(20); ImGui::Text(":"); ImGui::SameLine();
            ImGui::InputDouble("##constatorque", &coefa, NULL, NULL, "%.1f");
            ImGui::SameLine(); ImGui::Text("x 10 ^"); ImGui::SameLine();
            ImGui::InputInt("##constatorquepower", &coefapower, NULL, NULL);
            ImGui::AlignTextToFramePadding();
            ImGui::Text("b"); ImGui::SameLine(20); ImGui::Text(":"); ImGui::SameLine();
            ImGui::InputDouble("##constbtorque", &coefb, NULL, NULL, "%.1f");
            ImGui::SameLine(); ImGui::Text("x 10 ^"); ImGui::SameLine();
            ImGui::InputInt("##constbtorquepower", &coefbpower, NULL, NULL);
            ImGui::AlignTextToFramePadding();
            ImGui::Text("c"); ImGui::SameLine(20); ImGui::Text(":"); ImGui::SameLine();
            ImGui::InputDouble("##constctorque", &coefctorque, NULL, NULL, "%.1f");
            ImGui::PopItemWidth();
            */
            ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
            if (ImGui::BeginTabBar("Torque Option Bar", tab_bar_flags))
            {
                static bool show_leading_button = true;

                // Leading TabItemButton(): hover the "?" button to open a menu
                if (show_leading_button)
                    if (ImGui::TabItemButton("?", ImGuiTabItemFlags_Leading | ImGuiTabItemFlags_NoTooltip))
                        ImGui::OpenPopup("Torque Help Menu");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Torque Input Option!");

                if (ImGui::BeginTabItem("Equation"))
                {
                    torqueoption[0] = true;
                    torqueoption[1] = false;

                    ImGui::PushItemWidth(50);
                    ImGui::Text("aw^2 + bw + c , where w is in rpm");
                    ImGui::AlignTextToFramePadding();
                    ImGui::Text("a"); ImGui::SameLine(20); ImGui::Text(":"); ImGui::SameLine();
                    ImGui::InputDouble("##constatorque", &coefa, NULL, NULL, "%.1f");
                    ImGui::SameLine(); ImGui::Text("x 10 ^"); ImGui::SameLine();
                    ImGui::InputInt("##constatorquepower", &coefapower, NULL, NULL);
                    ImGui::AlignTextToFramePadding();
                    ImGui::Text("b"); ImGui::SameLine(20); ImGui::Text(":"); ImGui::SameLine();
                    ImGui::InputDouble("##constbtorque", &coefb, NULL, NULL, "%.1f");
                    ImGui::SameLine(); ImGui::Text("x 10 ^"); ImGui::SameLine();
                    ImGui::InputInt("##constbtorquepower", &coefbpower, NULL, NULL);
                    ImGui::AlignTextToFramePadding();
                    ImGui::Text("c"); ImGui::SameLine(20); ImGui::Text(":"); ImGui::SameLine();
                    ImGui::InputDouble("##constctorque", &coefctorque, NULL, NULL, "%.1f");
                    ImGui::PopItemWidth();

                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("Data"))
                {
                    torqueoption[0] = false;
                    torqueoption[1] = true;

                    ImGui::PushItemWidth(50);
                    ImGui::AlignTextToFramePadding();
                    ImGui::Text("Maximum Engine Torque"); ImGui::SameLine(175); ImGui::Text(":"); ImGui::SameLine();
                    ImGui::InputDouble("##valuetorquemax", &torquemax, NULL, NULL, "%.1f");
                    ImGui::SameLine(); ImGui::Text("Nm");
                    ImGui::SameLine(270); ImGui::Text("@"); ImGui::SameLine();
                    ImGui::InputDouble("##torquespeedmax", &torquespeed, NULL, NULL, "%1.f");
                    ImGui::SameLine(); ImGui::Text("rpm");

                    ImGui::AlignTextToFramePadding();
                    ImGui::Text("Maximum Engine Power"); ImGui::SameLine(175); ImGui::Text(":"); ImGui::SameLine();
                    ImGui::InputDouble("##valuepowermax", &powermax, NULL, NULL, "%.1f");
                    ImGui::SameLine(); ImGui::Text("kW");
                    ImGui::SameLine(270); ImGui::Text("@"); ImGui::SameLine();
                    ImGui::InputDouble("##powerspeedmax", &powerspeed, NULL, NULL, "%1.f");
                    ImGui::SameLine(); ImGui::Text("rpm");
                    ImGui::PopItemWidth();

                    ImGui::EndTabItem();
                }

                ImGui::EndTabBar();
                
            }

        }

        ImGui::End();

        ImGui::Begin("Gear Ratio Result");
        //------------------------------------------------------
        // Lowest Gear Ratio Variables
        //------------------------------------------------------
        static double Nfront, Nrear, lowestgear, maxtorque;
        static double dTorque, torqueenginespeed;
        //static double coordmaxtorque, coordlowestgear;

        //------------------------------------------------------
        // Intermediate Gear Ratio Variables
        //------------------------------------------------------
        static double constantgp, constantfactor = 0.9f, cgppower, cpower, lhgearratio;
        static double constantgear[10], progintgear[10];
        static double geomintgear[10];
        static double intgear[10];

        //------------------------------------------------------
        // Highest Gear Ratio Variables
        //------------------------------------------------------
        static double constatorque, constbtorque, constctorque, highestgear;
        static double maxpowerone, maxpowertwo, maxpower, powerenginespeedone,
            powerenginespeedtwo, powerenginespeed, discriminant;
        static double speedlimit;
        //static double coordmaxpower, coordhighestgear;
        //static double cospeedlimit;

        static double tractionforce, desiredpower, desiredenginespeed, desiredhighestgear;
        //static double coorddesiredhighestgear;

        //------------------------------------------------------
        // Lowest Gear Ratio Calculation
        //------------------------------------------------------
        //static double torque;
        Nfront = ((frontweight / 100 + hlratio * rollingresistance) /
            (1 + roadadhesion * hlratio) * weight);
        Nrear = ((rearweight / 100 - hlratio * rollingresistance) /
            (1 - roadadhesion * hlratio) * weight);
        
        if (torqueoption[0])
        {
            // torque = (constatorque * pow(enginespeed, 2)) + (constbtorque * enginespeed) + constctorque;
            // dTorque = (2 * constatorque * enginespeed) + constbtorque;
            dTorque = 0; // maximum torque where gradient of eqution is 0
            torqueenginespeed = (dTorque - coefbtorque) / (2 * coefatorque);
            maxtorque = (coefatorque * pow(torqueenginespeed, 2)) + (coefbtorque * torqueenginespeed) + coefctorque;
        }
        else if (torqueoption[1])
        {
            maxtorque = torquemax;
        }
        /*
        for (int i = 0; i < numpoints; i++)
        {
            if (coordmaxtorque < coordy[i])
            {
                coordmaxtorque = coordy[i];
            }

        }*/

        if (selected[0])
        {
            lowestgear = roadadhesion * wheelradius / (drivelineefficiency / 100) /
                maxtorque * Nfront;
            //coordlowestgear = roadadhesion * wheelradius / (drivelineefficiency / 100) /
            //    coordmaxtorque * Nfront;
        }
        else if (selected[1])
        {
            lowestgear = roadadhesion * wheelradius / (drivelineefficiency / 100) /
                maxtorque * Nrear;
            //coordlowestgear = roadadhesion * wheelradius / (drivelineefficiency / 100) /
            //    coordmaxtorque * Nrear;
        }
        
        /*else if (selected[2])
        {
            lowestgear = 1;
        }
        else
        {
            lowestgear = 0;
        }*/

        //------------------------------------------------------
        // Highest Gear Ratio Calculation - Maximum Speed
        //------------------------------------------------------
        //static double dpower;
        //dpower = (3 * constatorque * pow(powerenginespeed, 2)) + (2 * constbtorque * powerenginespeed) +
        //    constctorque;
        constatorque = 3 * coefatorque;
        constbtorque = 2 * coefbtorque;
        constctorque = coefctorque;
        discriminant = constbtorque * constbtorque - 4 * constatorque * constctorque;

        //Testing
        //ImGui::Text("a: %.10f", constatorque);
        //ImGui::Text("b: %.10f", constbtorque);
        //ImGui::Text("c: %.10f", constctorque);
        //ImGui::Text("Discriminant: %.10f", discriminant);
        if (discriminant > 0)
        {
            powerenginespeedone = (-constbtorque + sqrt(discriminant)) / (2 * constatorque);
            powerenginespeedtwo = (-constbtorque - sqrt(discriminant)) / (2 * constatorque);
            maxpowerone = ((coefatorque * pow(powerenginespeedone, 2)) + (coefbtorque * powerenginespeedone) +
                coefctorque) * (powerenginespeedone * 2 * PIvalue / 60);
            maxpowertwo = ((coefatorque * pow(powerenginespeedtwo, 2)) + (coefbtorque * powerenginespeedtwo) +
                coefctorque) * (powerenginespeedtwo * 2 * PIvalue / 60);
            if (powerenginespeedone > 0 && powerenginespeedtwo > 0)
            {
                if (maxpowerone > maxpowertwo)
                {
                    maxpower = maxpowerone;
                    powerenginespeed = powerenginespeedone;
                }
                else
                {
                    maxpower = maxpowertwo;
                    powerenginespeed = powerenginespeedone;
                }
            }
            else if (powerenginespeedone > 0 && powerenginespeedtwo <= 0)
            {
                powerenginespeed = powerenginespeedone;
                maxpower = maxpowerone;
            }
            else if (powerenginespeedone <= 0 && powerenginespeedtwo > 0)
            {
                powerenginespeed = powerenginespeedtwo;
                maxpower = maxpowertwo;
            }
            else
                ;


            //Testing
            //ImGui::Text("Engine Speed 1: %.2f", powerenginespeedone);
            //ImGui::Text("Engine Speed 2: %.2f", powerenginespeedtwo);
            //ImGui::Text("Engine Speed: %.2f", powerenginespeed);
            //ImGui::Text("Maximum Power 1: %.2f", maxpowerone);
            //ImGui::Text("Maximum Power 2: %.2f", maxpowertwo);
            //ImGui::Text("Maximum Power: %.2f", maxpower);
        }
        else if (discriminant == 0)
        {
            powerenginespeed = -constbtorque / (2 * constatorque);
            maxpower = ((constatorque * pow(powerenginespeed, 2)) + (constbtorque * powerenginespeed) +
                constctorque) * powerenginespeed * 2 * PIvalue / 60;
            //Testing
            //ImGui::Text("Engine Speed: %.2f", powerenginespeed);
            //ImGui::Text("Maximum Power: %.2f", maxpower);
        }
        else
        {
            powerenginespeed = 0;
        }

        if (powerenginespeed > maxenginespeedrpm)
        {
            powerenginespeed = maxenginespeedrpm;
            maxpower = ((coefatorque * pow(powerenginespeed, 2)) + (coefbtorque * powerenginespeed) +
                coefctorque) * (powerenginespeed * 2 * PIvalue / 60);
        }

        static double constaspeed, constbspeed, constcspeed, constdspeed, coefbb, coefcc, coefdd;
        static double root1, root2, root3, root2im = 0, root3im = 0;
        static double q, r, discrim, r13, term1, s, t, dum1;

        constaspeed = overallaerodynamic;
        constbspeed = 0;
        constcspeed = rollingresistance * weight;
        if (torqueoption[0])
        {
            constdspeed = maxpower * -1;
        }
        else if (torqueoption[1])
        {
            constdspeed = powermax * 1000 * -1;
        }
        
        //Testing
        //ImGui::InputDouble("coefa", &coefa, NULL, NULL, "%0.2f");
        //ImGui::InputDouble("coefb", &coefb, NULL, NULL, "%0.2f");
        //ImGui::InputDouble("coefc", &coefc, NULL, NULL, "%0.2f");
        //ImGui::InputDouble("coefd", &coefd, NULL, NULL, "%0.2f");
        //ImGui::Text("coefa: %.2f", constaspeed);
        //ImGui::Text("coefb: %.2f", constbspeed);
        //ImGui::Text("coefc: %.2f", constcspeed);
        //ImGui::Text("coefd: %.2f", constdspeed);

        coefbb = constbspeed / constaspeed;
        coefcc = constcspeed / constaspeed;
        coefdd = constdspeed / constaspeed;
        q = (3.0 * coefcc - pow(coefbb, 2.0)) / 9.0;
        r = (-27.0 * coefdd + coefbb * (9.0 * coefcc - 2.0 * pow(coefbb, 2.0))) / 54.0;
        discrim = pow(q, 3.0) + pow(r, 2.0);
        term1 = coefbb / 3.0;
        //ImGui::Text("coefbb: %.2f", coefbb);
        //ImGui::Text("coefcc: %.2f", coefcc);
        //ImGui::Text("coefdd: %.2f", coefdd);

        if (discrim < 0)
        {
            q = -q;
            dum1 = pow(q, 3);
            dum1 = acos(r / sqrt(dum1));
            r13 = 2.0 * sqrt(q);
            root1 = -term1 + r13 * cos(dum1 / 3.0);
            root2 = -term1 + r13 * cos((dum1 + (2.0 * PIvalue)) / 3.0);
            root3 = -term1 + r13 * cos((dum1 + (4.0 * PIvalue)) / 3.0);
            root2im = 0;
            root3im = 0;
        }
        else if (discrim > 0)
        {
            s = r + sqrt(discrim);
            if (s < 0)
            {
                s = -pow(-s, 1.0 / 3.0);
            }
            else
            {
                s = pow(s, 1.0 / 3.0);
            }
            t = r - sqrt(discrim);
            if (t < 0)
            {
                t = -pow(-t, 1.0 / 3.0);
            }
            else
            {
                t = pow(t, 1.0 / 3.0);
            }
            root1 = -term1 + s + t;
            term1 = term1 + (s + t) / 2.0;
            root3im = -term1;
            root2im = -term1;
            term1 = sqrt(3.0) * ((-t + s) / 2.0);
            root2 = term1;
            root3 = -term1;
        }
        else
        {
            if (r < 0)
            {
                r13 = -pow(-r, 1.0 / 3.0);
            }
            else
            {
                r13 = pow(r, 1.0 / 3.0);
            }
            root1 = -term1 + 2.0 * r13;
            root2 = -(r13 + term1);
            root3 = root2;
            root2im = 0;
            root3im = 0;
        }

        //Testing
        //ImGui::Text("q: %.2f", q);
        //ImGui::Text("r: %.2f", r);
        //ImGui::Text("r13: %.2f", r13);
        //ImGui::Text("term1: %.2f", term1);
        //ImGui::Text("Discirmination: %.2f", discrim);
        //ImGui::Text("Root 1: %.2f", root1);
        //ImGui::Text("Root 2: %.2f", root2); ImGui::SameLine(); ImGui::Text("%.2f", root2im);
        //ImGui::Text("Root 3: %.2f", root3); ImGui::SameLine(); ImGui::Text("%.2f", root2im);

        static double speeds[3];
        speeds[0] = root1;
        speeds[1] = root2;
        speeds[2] = root3;
        if (root2im > 0 || root2im < 0)
        {
            speedlimit = root1;
        }
        else
        {
            speedlimit = 0;
            for (int i = 0; i < 3; i++)
            {
                if (speedlimit < speeds[i])
                {
                    speedlimit = speeds[i];
                }
                //ImGui::Text("Speed: %.2f", speeds[i]);    //Testing
            }
        }
        //ImGui::Text("Vehicle Speed Limit: %.2f", speedlimit); //Testing

        if (torqueoption[0])
        {
            highestgear = wheelradius * (powerenginespeed * 2 * PIvalue / 60) / speedlimit;
        }
        else if (torqueoption[1])
        {
            highestgear = wheelradius * (powerspeed * 2 * PIvalue / 60) / speedlimit;
        }
        

        // For Coordinate
        /*
        coordmaxpower = 0;
        for (int i = 0; i < numpoints; i++)
        {
            static double indexpower;
            indexpower = coordy[i] * coordx[i] * 2.f * PIvalue / 60.f;
            if (coordmaxpower < indexpower)
            {
                coordmaxpower = indexpower;
            }
        }

        for (int i = 0; i < numpoints; i++)
        {
            coordpower[i] = coordy[i] * coordx[i] * 2.f * PIvalue / 60.f;
        }

        static double coconstaspeed, coconstbspeed, coconstcspeed, coconstdspeed, cocoefbb, cocoefcc, cocoefdd;
        static double coroot1, coroot2, coroot3, coroot2im = 0, coroot3im = 0;
        static double coq, cor, codiscrim, cor13, coterm1, coss, cot, codum1;

        coconstaspeed = overallaerodynamic;
        coconstbspeed = 0;
        coconstcspeed = rollingresistance * weight;
        coconstdspeed = coordmaxpower * -1;

        cocoefbb = coconstbspeed / coconstaspeed;
        cocoefcc = coconstcspeed / coconstaspeed;
        cocoefdd = coconstdspeed / coconstaspeed;
        coq = (3.0 * cocoefcc - pow(cocoefbb, 2.0)) / 9.0;
        cor = (-27.0 * cocoefdd + cocoefbb * (9.0 * cocoefcc - 2.0 * pow(cocoefbb, 2.0))) / 54.0;
        codiscrim = pow(coq, 3.0) + pow(cor, 2.0);
        coterm1 = cocoefbb / 3.0;

        if (codiscrim < 0)
        {
            coq = -coq;
            codum1 = pow(coq, 3);
            codum1 = acos(cor / sqrt(codum1));
            cor13 = 2.0 * sqrt(coq);
            coroot1 = -coterm1 + cor13 * cos(codum1 / 3.0);
            coroot2 = -coterm1 + cor13 * cos((codum1 + (2.0 * PIvalue)) / 3.0);
            coroot3 = -coterm1 + cor13 * cos((codum1 + (4.0 * PIvalue)) / 3.0);
            coroot2im = 0;
            coroot3im = 0;
        }
        else if (codiscrim > 0)
        {
            coss = cor + sqrt(codiscrim);
            if (coss < 0)
            {
                coss = -pow(-coss, 1.0 / 3.0);
            }
            else
            {
                coss = pow(coss, 1.0 / 3.0);
            }
            cot = cor - sqrt(codiscrim);
            if (cot < 0)
            {
                cot = -pow(-cot, 1.0 / 3.0);
            }
            else
            {
                cot = pow(cot, 1.0 / 3.0);
            }
            coroot1 = -coterm1 + coss + cot;
            coterm1 = coterm1 + (coss + cot) / 2.0;
            coroot3im = -coterm1;
            coroot2im = -coterm1;
            coterm1 = sqrt(3.0) * ((-cot + coss) / 2.0);
            coroot2 = coterm1;
            coroot3 = -coterm1;
        }
        else
        {
            if (cor < 0)
            {
                cor13 = -pow(-cor, 1.0 / 3.0);
            }
            else
            {
                cor13 = pow(cor, 1.0 / 3.0);
            }
            coroot1 = -coterm1 + 2.0 * cor13;
            coroot2 = -(cor13 + coterm1);
            coroot3 = coroot2;
            coroot2im = 0;
            coroot3im = 0;
        }

        static double cospeeds[3];
        cospeeds[0] = coroot1;
        cospeeds[1] = coroot2;
        cospeeds[2] = coroot3;
        if (coroot2im > 0 || coroot2im < 0)
        {
            cospeedlimit = coroot1;
        }
        else
        {
            cospeedlimit = 0;
            for (int i = 0; i < 3; i++)
            {
                if (cospeedlimit < cospeeds[i])
                {
                    cospeedlimit = cospeeds[i];
                }
            }
        }

        coordhighestgear = wheelradius * (powerenginespeed * 2 * PIvalue / 60) / cospeedlimit;
        */

        //------------------------------------------------------
        // Highest Gear Ratio Calculation - Desired Speed
        //------------------------------------------------------
        tractionforce = (rollingresistance * weight) + (overallaerodynamic * pow(vehiclespeed, 2));
        desiredpower = tractionforce * vehiclespeed;

        static double dconstaspeed, dconstbspeed, dconstcspeed, dconstdspeed, dcoefbb, dcoefcc, dcoefdd;
        static double droot1, droot2, droot3, droot2im = 0, droot3im = 0;
        static double dq, dr, ddiscrim, dr13, dterm1, ds, dt, ddum1;

        dconstaspeed = coefatorque ;
        dconstbspeed = coefbtorque ;
        dconstcspeed = coefctorque ;
        dconstdspeed = desiredpower * -1 / 2 / PIvalue * 60;

        dcoefbb = dconstbspeed / dconstaspeed;
        dcoefcc = dconstcspeed / dconstaspeed;
        dcoefdd = dconstdspeed / dconstaspeed;
        dq = (3.0 * dcoefcc - pow(dcoefbb, 2.0)) / 9.0;
        dr = (-27.0 * dcoefdd + dcoefbb * (9.0 * dcoefcc - 2.0 * pow(dcoefbb, 2.0))) / 54.0;
        ddiscrim = pow(dq, 3.0) + pow(dr, 2.0);
        dterm1 = dcoefbb / 3.0;
        /*ImGui::Text("coefa: %.9f", dconstaspeed);
        ImGui::Text("coefb: %.4f", dconstbspeed);
        ImGui::Text("coefc: %.2f", dconstcspeed);
        ImGui::Text("coefd: %.2f", dconstdspeed);
        ImGui::Text("power: %.2f", desiredpower);
        ImGui::Text("coefbb: %.2f", dcoefbb);
        ImGui::Text("coefcc: %.2f", dcoefcc);
        ImGui::Text("coefdd: %.2f", dcoefdd);
        ImGui::Text("r: %.2f", dr);
        ImGui::Text("q: %.2f", dq);
        ImGui::Text("discrim: %.2f", ddiscrim);
        ImGui::Text("term1: %.2f", term1);*/

        if (ddiscrim < 0)
        {
            dq = -dq;
            ddum1 = pow(dq, 3);
            //ImGui::Text("ddum1: %.2f", ddum1);
            //ImGui::Text("ddum1: %.2f", dr / sqrt(ddum1));
            ddum1 = acos(dr / sqrt(ddum1));
            //ImGui::Text("ddum1: %.2f", ddum1);
            dr13 = 2.0 * sqrt(dq);
            droot1 = -dterm1 + dr13 * cos(ddum1 / 3.0);
            droot2 = -dterm1 + dr13 * cos((ddum1 + (2.0 * PIvalue)) / 3.0);
            droot3 = -dterm1 + dr13 * cos((ddum1 + (4.0 * PIvalue)) / 3.0);
            droot2im = 0;
            droot3im = 0;
        }
        else if (ddiscrim > 0)
        {
            ds = dr + sqrt(ddiscrim);
            if (s < 0)
            {
                ds = -pow(-ds, 1.0 / 3.0);
            }
            else
            {
                ds = pow(ds, 1.0 / 3.0);
            }
            dt = dr - sqrt(ddiscrim);
            if (t < 0)
            {
                dt = -pow(-dt, 1.0 / 3.0);
            }
            else
            {
                dt = pow(dt, 1.0 / 3.0);
            }
            droot1 = -dterm1 + ds + dt;
            dterm1 = dterm1 + (ds + dt) / 2.0;
            droot3im = -dterm1;
            droot2im = -dterm1;
            dterm1 = sqrt(3.0) * ((-dt + ds) / 2.0);
            droot2 = dterm1;
            droot3 = -dterm1;
        }
        else
        {
            if (dr < 0)
            {
                dr13 = -pow(-dr, 1.0 / 3.0);
            }
            else
            {
                dr13 = pow(dr, 1.0 / 3.0);
            }
            droot1 = -dterm1 + 2.0 * dr13;
            droot2 = -(dr13 + dterm1);
            droot3 = droot2;
            droot2im = 0;
            droot3im = 0;
        }

        static double dgear[3] = { 0 };
        
        if (droot1 < maxenginespeedrpm && droot2 < maxenginespeedrpm && droot3 < maxenginespeedrpm)
        {
            dgear[0] = wheelradius * (droot1 * 2 * PIvalue / 60) / vehiclespeed;
            dgear[1] = wheelradius * (droot2 * 2 * PIvalue / 60) / vehiclespeed;
            dgear[2] = wheelradius * (droot3 * 2 * PIvalue / 60) / vehiclespeed;
        }
        else if (droot1 > maxenginespeedrpm && droot2 < maxenginespeedrpm && droot3 < maxenginespeedrpm)
        {
            dgear[1] = wheelradius * (droot2 * 2 * PIvalue / 60) / vehiclespeed;
            dgear[2] = wheelradius * (droot3 * 2 * PIvalue / 60) / vehiclespeed;
        }
        else if (droot1 < maxenginespeedrpm && droot2 > maxenginespeedrpm && droot3 < maxenginespeedrpm)
        {
            dgear[0] = wheelradius * (droot1 * 2 * PIvalue / 60) / vehiclespeed;
            dgear[2] = wheelradius * (droot3 * 2 * PIvalue / 60) / vehiclespeed;
        }
        else if (droot1 < maxenginespeedrpm && droot2 < maxenginespeedrpm && droot3 > maxenginespeedrpm)
        {
            dgear[0] = wheelradius * (droot1 * 2 * PIvalue / 60) / vehiclespeed;
            dgear[1] = wheelradius * (droot2 * 2 * PIvalue / 60) / vehiclespeed;
        }
        else if (droot1 > maxenginespeedrpm && droot2 > maxenginespeedrpm && droot3 < maxenginespeedrpm)
        {
            dgear[2] = wheelradius * (droot3 * 2 * PIvalue / 60) / vehiclespeed;
        }
        else if (droot1 > maxenginespeedrpm && droot2 < maxenginespeedrpm && droot3 > maxenginespeedrpm)
        {
            dgear[1] = wheelradius * (droot2 * 2 * PIvalue / 60) / vehiclespeed;
        }
        else if (droot1 < maxenginespeedrpm && droot2 > maxenginespeedrpm && droot3 > maxenginespeedrpm)
        {
            dgear[0] = wheelradius * (droot1 * 2 * PIvalue / 60) / vehiclespeed;
        }
        else
        {

        }
        
        if (droot2im > 0 || droot2im < 0)
        {
            desiredhighestgear = dgear[0];
        }
        else
        {
            desiredhighestgear = 100;
            for (int i = 0; i < 3; i++)
            {
                if (desiredhighestgear > dgear[i])
                {
                    if (dgear[i] > 0)
                    {
                        desiredhighestgear = dgear[i];

                    }
                }
            }
        }

        //static double coordpowerspeed;
        

        static bool methodsgear[3] = { true, false, false };
        ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
        if (ImGui::BeginTabBar("MyTabBar", tab_bar_flags))
        {
            static bool show_leading_button = true;

            // Leading TabItemButton(): hover the "?" button to open a menu
            if (show_leading_button)
                if (ImGui::TabItemButton("?", ImGuiTabItemFlags_Leading | ImGuiTabItemFlags_NoTooltip))
                    ImGui::OpenPopup("MyHelpMenu");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Method on calculating gear ratio!");

            if (ImGui::BeginTabItem("Progressive"))
            {
                methodsgear[0] = true;
                methodsgear[1] = false;
                //methodsgear[2] = false;
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Geometric"))
            {
                methodsgear[0] = false;
                methodsgear[1] = true;
                //methodsgear[2] = false;
                ImGui::EndTabItem();
            }
            /*if (ImGui::BeginTabItem("Combination"))
            {
                methodsgear[0] = false;
                methodsgear[1] = false;
                methodsgear[2] = true;
                ImGui::EndTabItem();
            }*/
            ImGui::EndTabBar();
        }

        // highest gear ratio selection
        static char* typehighestgear[2] = { "Vehicle Speed Limit", "Desired Maximum Vehicle Speed" };
        static bool selectedgear[2] = { true, false };

        ImGui::Text("Highest Gear Ratio Type:");
        if (torqueoption[0])
        {
            for (int i = 0; i < 2; i++)
            {

                if (ImGui::Checkbox(typehighestgear[i], &selectedgear[i]))
                {
                    switch (i)
                    {
                    case 0:
                        selectedgear[1] = false;
                        break;
                    case 1:
                        selectedgear[0] = false;
                        break;
                    }
                }

                if (i == 0)
                {
                    ImGui::SameLine();
                    ImGui::TextDisabled("(?)");
                    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
                    {
                        ImGui::BeginTooltip();
                        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
                        ImGui::TextUnformatted("The limit of vehicle speed, engine can achieved");
                        ImGui::PopTextWrapPos();
                        ImGui::EndTooltip();
                    }
                }
            }
        }
        else if (torqueoption[1])
        {
            ImGui::Checkbox(typehighestgear[0], &selectedgear[0]);
            ImGui::SameLine();
            ImGui::TextDisabled("(?)");
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
            {
                ImGui::BeginTooltip();
                ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
                ImGui::TextUnformatted("The limit of vehicle speed, engine can achieved");
                ImGui::PopTextWrapPos();
                ImGui::EndTooltip();
            }
        }
        

        //------------------------------------------------------
        // Highest Gear Ratio Calculation - Desired Speed
        //------------------------------------------------------
        //static double coordlhgearratio, coordconstantgp;
        //static double coordconstantgear[10], coordprogintgear[10];
        //static double coordintgear[10], coordgeointgear[10];

        cgppower = 1.0 / (gearnumber - 1.0);
        if (selectedgear[0])
        {
            lhgearratio = lowestgear / highestgear;
            //coordlhgearratio = coordlowestgear / coordhighestgear;
        }
        else if (selectedgear[1])
        {
            lhgearratio = lowestgear / desiredhighestgear;
        }
        else
            ;
        cpower = 1.0 - (gearnumber / 2.0);
        constantgp = pow(lhgearratio, cgppower);
        //coordconstantgp = pow(coordlhgearratio, cgppower);

        if (methodsgear[0])
        {
            ImGui::PushItemWidth(50);
            ImGui::AlignTextToFramePadding();
            ImGui::Text("Constant Factor"); ImGui::SameLine(110); ImGui::Text(":"); ImGui::SameLine();
            ImGui::InputDouble("##constantfactor", &constantfactor, NULL, NULL, "%.1f");
            ImGui::PopItemWidth();

            constantgear[0] = constantgp * pow(constantfactor, cpower);
            //coordconstantgear[0] = coordconstantgp * pow(constantfactor, cpower);
            for (int i = 1; i < (gearnumber - 2); i++)
            {
                constantgear[i] = constantfactor * constantgear[i - 1];
                //coordconstantgear[i] = constantfactor * coordconstantgear[i - 1];
            }

            progintgear[0] = lowestgear;
            //coordprogintgear[0] = coordlowestgear;
            for (int i = 0; i < (gearnumber - 2); i++)
            {
                progintgear[i + 1] = progintgear[i] / constantgear[i];
                //coordprogintgear[i + 1] = coordprogintgear[i] / constantgear[i];
            }

            for (int i = 0; i < (gearnumber - 1); i++)
            {
                intgear[i] = progintgear[i];
                //coordintgear[i] = coordprogintgear[i];
            }

        }
        if (methodsgear[1])
        {
            geomintgear[0] = lowestgear;
            //coordgeointgear[0] = coordlowestgear;
            for (int i = 0; i < (gearnumber - 2); i++)
            {
                geomintgear[i + 1] = geomintgear[i] / constantgp;
                //coordgeointgear[i + 1] = coordgeointgear[i] / coordconstantgp;
            }

            for (int i = 0; i < (gearnumber - 1); i++)
            {
                intgear[i] = geomintgear[i];
                //coordintgear[i] = coordgeointgear[i];
            }
        }

        if (selectedgear[0])
        {
            intgear[gearnumber - 1] = highestgear;
            //coordintgear[gearnumber - 1] = coordhighestgear;
        }
        else if (selectedgear[1])
        {
            intgear[gearnumber - 1] = desiredhighestgear;
        }

        //------------------------------------------------------
        // Engine WOT Power/Torque vs Engine Speed Graph Calculation
        //------------------------------------------------------
        static double torque[1000], enginespeed[1000], power[1000];  // engine speed unit in rpm
        static double maxtorquepoint[1], enginespeedpoint[1], enginespeedlimit;
        static int maxyaxis = 700;
        static int countgraph;

        static double coordtorque[1000], coordenginespeed[1000], coordenginepower[1000];

        countgraph = maxenginespeedrpm / 10 + 1;
        for (int i = 0; i < 1000; i++)
        {
            enginespeed[i] = i * 10.0 * 0.001;
            torque[i] = (coefatorque * pow(enginespeed[i] * 1000.0, 2)) +
                (coefbtorque * enginespeed[i] * 1000.0) + coefctorque;
            power[i] = torque[i] * (enginespeed[i] * 1000.0 * 2.0 * PIvalue / 60.0) / 1000;
        }
        for (int i = 0; i < numpoints; i++)
        {
            coordenginespeed[i] = coordx[i];
            coordtorque[i] = coordy[i];
            coordenginepower[i] = coordtorque[i] * (coordenginespeed[i] * 1000.0 * 2.0 * PIvalue / 60.0) / 1000;
        }

        //------------------------------------------------------
        // Engine Speed vs Vehicle Speed Graph Calculation
        //------------------------------------------------------
        static double enginespeedgraph[10][2] = { 0.f }, vehiclespeedgraph[10][2] = { 0.f };  // engine speed unit in rpm
        static double dotlinex[10][2] = { 0.f }, dotliney[10][2] = { 0.f };
        static double pointy[20], pointx[20];
        static double verticleline[20];
        static double velocitydiff[10];
        static int maxyaxisgear = 10000, maxxaxisgear = 100;
        static int countgeargraph;
        const char* numberofgear[10] = { "n1", "n2", "n3", "n4", "n5", "n6", "n7", "n8", "n9", "n10" };
        const char* dotlinename[10] = { "##d1", "##d2", "##d3", "##d4", "##d5", "##d6", "##d7",
            "##d8", "##d9", "##d10" };
        //const char* verticleline[10] = { "##v1", "##v2", "##v3", "##v4", "##v5", "##v6", "##v7",
        //    "##v8", "##v9", "##v10" };

        countgeargraph = gearnumber * 2;

        if (selectedgear[0])
        {
            enginespeedgraph[gearnumber-1][1] = speedlimit * ((intgear[gearnumber - 1] / wheelradius) * 60. / 2. / PIvalue * 0.001);
            vehiclespeedgraph[gearnumber-1][1] = speedlimit * 0.001 * 60. * 60.;

            if (enginespeedgraph[gearnumber - 1][1] > (maxenginespeedrpm * 0.001))
            {
                enginespeedgraph[gearnumber - 1][1] = maxenginespeedrpm * 0.001;
                vehiclespeedgraph[gearnumber - 1][1] = (maxenginespeedrpm * 2. * PIvalue / 60.) * wheelradius / intgear[gearnumber - 1] * 0.001 * 60. * 60.;
            }
        }
        if (selectedgear[1])
        {
            enginespeedgraph[gearnumber-1][1] = vehiclespeed * ((intgear[gearnumber - 1] / wheelradius) * 60. / 2. / PIvalue * 0.001);
            vehiclespeedgraph[gearnumber - 1][1] = vehiclespeed * 0.001 * 60. * 60.;

            if (enginespeedgraph[gearnumber - 1][1] > (maxenginespeedrpm * 0.001))
            {
                enginespeedgraph[gearnumber - 1][1] = maxenginespeedrpm * 0.001;
                vehiclespeedgraph[gearnumber - 1][1] = (maxenginespeedrpm * 2. * PIvalue / 60.) * wheelradius / intgear[gearnumber - 1] * 0.001 * 60. * 60.;
            }
        }

        for (int i = (gearnumber - 2), j = 1; i >= 0; i--)
        {
            enginespeedgraph[i][j] = enginespeedgraph[gearnumber - 1][1];
            vehiclespeedgraph[i][j] = (wheelradius / intgear[i]) * (enginespeedgraph[i][j] * 1000. * 2. * PIvalue / 60.) * 0.001 * 60. * 60.;

        }

        for (int i = 0; i < gearnumber; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                if (j == 0)
                {
                    dotlinex[i][j] = 0.0;
                    dotliney[i][j] = 0.0;
                }
                else
                {
                    dotlinex[i][j] = vehiclespeedgraph[i][j];
                    dotliney[i][j] = enginespeedgraph[i][j];
                }

            }
        }

        for (int i = 1, j = 0, k = 0; i < gearnumber; i++, k++)
        {
            enginespeedgraph[i][j] = (intgear[i] / wheelradius) * (vehiclespeedgraph[k][j + 1] * 60. / 2. / PIvalue / 0.001 / 60. / 60.) * 0.001;
            vehiclespeedgraph[i][j] = vehiclespeedgraph[k][j + 1];
        }

        for (int i = 0, j = 1; i < gearnumber; i++)
        {
            verticleline[i] = vehiclespeedgraph[i][j];
        }

        for (int i = 0; i < (gearnumber); i++)
        {
            velocitydiff[i] = vehiclespeedgraph[i][1] - vehiclespeedgraph[i][0];
        }

        //------------------------------------------------------
        // Wheel Torque/Power vs Vehicle Speed Graph Calculation
        //------------------------------------------------------

        static double torquewheel[10][1000], powerengine[10][1000], wheelvelocity[10][1000];
        static double roadresistance[1000], velocitygraph[1000];
        static int countvelocitylimit[10], countroad;
        
        for (int i = 0; i < gearnumber; i++)
        {
            for (int j = 0; j < 1000; j++)
            {
                wheelvelocity[i][j] = 0;
                torquewheel[i][j] = 0;
                powerengine[i][j] = 0;
            }

            for (int j = 0, k = 0; j < 1000; j++)
            {
                if (selectedgear[0] == true /* && enginespeed[j] > enginespeedgraph[i][0] */ && enginespeed[j] < (enginespeedgraph[i][1] + 2))
                {
                    wheelvelocity[i][k] = wheelradius * ((enginespeed[j] * 1000.0 * 2.0 * PIvalue / 60.0) / intgear[i]) * 60.0 * 60.0 / 1000.0;
                    torquewheel[i][k] = intgear[i] * torque[j] * (drivelineefficiency / 100);
                    powerengine[i][k] = power[j] * (drivelineefficiency / 100);
                    countvelocitylimit[i] = k + 1;
                    k++;
                    
                }
                else if (selectedgear[1] == true /* && enginespeed[j] > enginespeedgraph[i][0]*/ && enginespeed[j] < (enginespeedgraph[i][1] + 2))
                {
                    wheelvelocity[i][k] = wheelradius * ((enginespeed[j] * 1000.0 * 2.0 * PIvalue / 60.0) / intgear[i]) * 60.0 * 60.0 / 1000.0;
                    torquewheel[i][k] = intgear[i] * torque[j] * (drivelineefficiency / 100);
                    powerengine[i][k] = power[j] * (drivelineefficiency / 100);
                    countvelocitylimit[i] = k + 1;
                    k++;
                }                       
            }
        }
        
        velocitygraph[0] = 0.;
        roadresistance[0] = 0.;
        for (int i = 1, j = 1; i < 1000; i++)
        {
            if (selectedgear[0] && velocitygraph[i - 1] < (vehiclespeed * 60. * 60. * 0.001 + 30))
            {
                velocitygraph[i] = i * 0.1 * 60. * 60. * 0.001;
                roadresistance[i] = (rollingresistance * weight + overallaerodynamic *
                    pow((velocitygraph[i] / 60. / 60. / 0.001), 2)) * velocitygraph[i] / 60. / 60. / 0.001
                    * 0.001 * (drivelineefficiency / 100);
                countroad = j++;
            }
            else if (selectedgear[1] && velocitygraph[i - 1] < (vehiclespeed * 60. * 60. * 0.001 + 30))
            {
                velocitygraph[i] = i * 0.1 * 60. * 60. * 0.001;
                roadresistance[i] = (rollingresistance * weight + overallaerodynamic *
                    pow((velocitygraph[i] / 60. / 60. / 0.001), 2)) * velocitygraph[i] / 60. / 60. / 0.001
                    * 0.001 * (drivelineefficiency / 100);
                countroad = j++;
            }
            else
            {
                break;
            }
             

        }

        if (ImGui::CollapsingHeader("Gear Ratio", ImGuiTreeNodeFlags_None))
        {
            ImGui::Text("Lowest Gear Ratio: %.2f", lowestgear);

            if (methodsgear[0])
            {
                for (int i = 0; i < (gearnumber - 2); i++)
                {
                    ImGui::Text("Gear Ratio %i: %.2f", i + 2, progintgear[i + 1]);
                }
            }
            else if (methodsgear[1])
            {
                for (int i = 0; i < (gearnumber - 2); i++)
                {
                    ImGui::Text("Gear Ratio %i: %.2f", i + 2, geomintgear[i + 1]);
                }
            }

            if (selectedgear[0])
            {
                ImGui::Text("Highest Gear Ratio: %.2f", highestgear);
            }
            else if (selectedgear[1])
            {
                ImGui::Text("Highest Gear Ratio: %.2f", desiredhighestgear);
            }
            else
                ;
/*
            if (torqueoption[0])
            {
                if (methodsgear[0])
                {
                    ImGui::Text("Lowest Gear Ratio: %.2f", lowestgear);

                    for (int i = 0; i < (gearnumber - 2); i++)
                    {
                        ImGui::Text("Gear Ratio %i: %.2f", i + 2, progintgear[i + 1]);
                    }

                    if (selectedgear[0])
                    {
                        ImGui::Text("Highest Gear Ratio: %.2f", highestgear);
                    }
                    else if (selectedgear[1])
                    {
                        ImGui::Text("Highest Gear Ratio: %.2f", desiredhighestgear);
                    }
                    else
                        ;
                }
                if (methodsgear[1])
                {
                    ImGui::Text("Lowest Gear Ratio: %.2f", lowestgear);

                    for (int i = 0; i < (gearnumber - 2); i++)
                    {
                        ImGui::Text("Gear Ratio %i: %.2f", i + 2, geomintgear[i + 1]);
                    }

                    if (selectedgear[0])
                    {
                        ImGui::Text("Highest Gear Ratio: %.2f", highestgear);
                    }
                    else if (selectedgear[1])
                    {
                        ImGui::Text("Highest Gear Ratio: %.2f", desiredhighestgear);
                    }
                    else
                        ;
                }
            }
            else if (torqueoption[1])
            {
                if (methodsgear[0])
                {
                    ImGui::Text("Lowest Gear Ratio: %.2f", coordlowestgear);

                    for (int i = 0; i < (gearnumber - 2); i++)
                    {
                        ImGui::Text("Gear Ratio %i: %.2f", i + 2, coordprogintgear[i + 1]);
                    }

                    if (selectedgear[0])
                    {
                        ImGui::Text("Highest Gear Ratio: %.2f", coordhighestgear);
                    }
                    else if (selectedgear[1])
                    {
                        
                    }
                    else
                        ;
                }
                if (methodsgear[1])
                {
                    ImGui::Text("Lowest Gear Ratio: %.2f", coordlowestgear);

                    for (int i = 0; i < (gearnumber - 2); i++)
                    {
                        ImGui::Text("Gear Ratio %i: %.2f", i + 2, coordgeointgear[i + 1]);
                    }

                    if (selectedgear[0])
                    {
                        ImGui::Text("Highest Gear Ratio: %.2f", coordhighestgear);
                    }
                    else if (selectedgear[1])
                    {

                    }
                    else
                        ;
                }
            }*/
            

        }
        if (torqueoption[0])
        {
            if (ImGui::CollapsingHeader("Engine WOT Torque/Power vs Engine Speed Graph", ImGuiTreeNodeFlags_None))
            {
                maxtorquepoint[0] = maxtorque;
                enginespeedpoint[0] = torqueenginespeed * 0.001;
                static double maxxaxisengine;
                maxxaxisengine = ceil(maxenginespeedrpm / 1000.) + 1;
                if (maxtorque > (maxpower / 1000.0))
                {
                    maxyaxis = maxtorque + 50.0;
                }
                else
                {
                    maxyaxis = (maxpower / 1000.0) + 50.0;

                }


                ImPlot::CreateContext();
                if (ImPlot::BeginPlot("Engine WOT Torque/Power vs Engine Speed"))
                {
                    ImPlot::SetupAxes("Engine Speed (x1000 rpm)", "Torque (Nm) / Power (kW)");
                    ImPlot::SetupAxesLimits(0, maxxaxisengine, 0, maxyaxis);
                    ImPlot::PlotLine("Torque", enginespeed, torque, countgraph);
                    ImPlot::PlotLine("Power", enginespeed, power, countgraph);
                    ImPlot::EndPlot();
                }
                ImPlot::DestroyContext();
                ImGui::Text("Maximum Torque: (%1.f Nm, %1.f rpm)", maxtorquepoint[0], enginespeedpoint[0] * 1000);
                ImGui::Text("Maximum Power : (%1.f kW, %1.f rpm)", maxpower / 1000, powerenginespeed);
                /*
                            if (torqueoption[0])
                            {
                                maxtorquepoint[0] = maxtorque;
                                enginespeedpoint[0] = torqueenginespeed * 0.001;
                                if (maxtorque > (maxpower / 1000.0))
                                {
                                    maxyaxis = maxtorque + 50.0;
                                }
                                else
                                {
                                    maxyaxis = (maxpower / 1000.0) + 50.0;

                                }


                                ImPlot::CreateContext();
                                if (ImPlot::BeginPlot("Engine WOT Torque/Power-Speed"))
                                {
                                    ImPlot::SetupAxes("Engine Speed (x1000 rpm)", "Torque (Nm) / Power (kW)");
                                    ImPlot::SetupAxesLimits(0, 10, 0, maxyaxis);
                                    ImPlot::PlotLine("Torque", enginespeed, torque, countgraph);
                                    ImPlot::PlotLine("Power", enginespeed, power, countgraph);
                                    ImPlot::EndPlot();
                                }
                                ImPlot::DestroyContext();
                                ImGui::Text("Maximum Torque: (%1.f Nm, %1.f rpm)", maxtorquepoint[0], enginespeedpoint[0] * 1000);
                                ImGui::Text("Maximum Power : (%1.f kW, %1.f rpm)", maxpower / 1000, powerenginespeed);
                            }
                            else if (torqueoption[1])
                            {
                                if (coordmaxpower > (coordmaxpower / 1000.0))
                                {
                                    maxyaxis = coordmaxtorque + 50.0;
                                }
                                else
                                {
                                    maxyaxis = (coordmaxpower / 1000.0) + 50.0;

                                }


                                ImPlot::CreateContext();
                                if (ImPlot::BeginPlot("Engine WOT Torque/Power-Speed"))
                                {
                                    ImPlot::SetupAxes("Engine Speed (x1000 rpm)", "Torque (Nm) / Power (kW)");
                                    ImPlot::SetupAxesLimits(0, 10, 0, maxyaxis);
                                    ImPlot::PlotLine("Torque", coordenginespeed, coordtorque, countgraph);
                                    ImPlot::PlotLine("Power", coordenginespeed, coordenginepower, countgraph);
                                    ImPlot::EndPlot();
                                }
                                ImPlot::DestroyContext();
                            }
                  */
            }
        }
        if (ImGui::CollapsingHeader("Engine Speed vs Vehicle Speed Graph", ImGuiTreeNodeFlags_None))
        {
            /*
            for (int i = 0; i < gearnumber; i++)
            {
                ImGui::Text("gear ratio %i: %1.f", i, intgear[i]);
                for (int j = 0; j < 2; j++)
                {
                    ImGui::Text("engine speed %i %i: %1.f", i, j, enginespeedgraph[i][j]);
                    ImGui::Text("vehicle speed %i %i: %1.f", i, j, vehiclespeedgraph[i][j]);
                }
                
            }*/

            maxyaxisgear = speedlimit * ((intgear[gearnumber - 1] / wheelradius) * 60. / 2. / PIvalue * 0.001) + 1.;
            maxxaxisgear = vehiclespeedgraph[gearnumber - 1][1] + 20;
            ImPlot::CreateContext();
            if (ImPlot::BeginPlot("Engine Speed vs Vehicle Speed"))
            {
                ImPlot::SetupAxes("Vehicle Speed (km/h)", "Engine Speed (x1000 rpm)");
                ImPlot::SetupAxesLimits(0, maxxaxisgear, 0, maxyaxisgear);
                ImPlot::SetNextLineStyle(ImVec4(0.4f, 0.4f, 0.4f, 1.f));
                ImPlot::PlotInfLines("##vertical line", verticleline, gearnumber);
                for (int i = 0; i < gearnumber; i++)
                {
                    ImPlot::SetNextLineStyle(ImVec4(0.2f, 0.2f, 0.2f, 1.f));
                    ImPlot::PlotLine(dotlinename[i], dotlinex[i], dotliney[i], 2);
                    ImPlot::PlotLine(numberofgear[i], vehiclespeedgraph[i], enginespeedgraph[i], 2, ImPlotLineFlags_Segments);
                }

                ImPlot::EndPlot();
            }
            ImPlot::DestroyContext();

            for (int i = 0; i < (gearnumber); i++)
            {
                ImGui::Text("Vehicle Speed Range %i: %.1f km/h", i+1, velocitydiff[i]);
            }
        }
        if (torqueoption[0])
        {
            if (ImGui::CollapsingHeader("Torque vs Vehicle Speed Graph", ImGuiTreeNodeFlags_None))
            {
                ImPlot::CreateContext();
                if (ImPlot::BeginPlot("Torque vs Vehicle Speed"))
                {
                    const char* geartag[10] = { "n1##tag", "n2##tag", "n3##tag", "n4##tag", "n5##tag", "n6##tag",
                        "n7##tag", "n8##tag", "n9##tag", "n10##tag" };
                    static int yaxislimit = 100;
                    static int xaxislimit = 100;
                    static double maximumwheelvelocity;
                    yaxislimit = maxtorque * intgear[0] * (drivelineefficiency / 100) + 20;
                    xaxislimit = speedlimit * 60 * 60 / 1000 + 50;
                    maximumwheelvelocity = verticleline[gearnumber - 1];

                    ImPlot::SetupAxes("Vehicle Speed (km/h)", "Torque (Nm)");
                    ImPlot::SetupAxesLimits(0, xaxislimit, 0, yaxislimit);
                    ImPlot::SetNextLineStyle(ImVec4(0.4f, 0.4f, 0.4f, 1.f));
                    ImPlot::PlotInfLines("##vertical line torque", &maximumwheelvelocity, 1);
                    for (int i = 0; i < gearnumber; i++)
                    {
                        ImPlot::PlotLine(geartag[i], wheelvelocity[i], torquewheel[i], countvelocitylimit[i]);
                    }
                    ImPlot::EndPlot();
                }
                ImPlot::DestroyContext();
            }
            if (ImGui::CollapsingHeader("Power vs Vehicle Speed Graph", ImGuiTreeNodeFlags_None))
            {
                const char* geartag[10] = { "n1##tag", "n2##tag", "n3##tag", "n4##tag", "n5##tag", "n6##tag",
                        "n7##tag", "n8##tag", "n9##tag", "n10##tag" };
                static int yaxislimit = 100;
                static int xaxislimit = 100;
                static double maximumwheelvelocity, maximumvehiclepower;
                yaxislimit = maxpower * (drivelineefficiency / 100) / 1000 + 20;
                xaxislimit = speedlimit * 60 * 60 / 1000 + 50;
                maximumwheelvelocity = verticleline[gearnumber - 1];
                if (selectedgear[0])
                {
                    maximumvehiclepower = maxpower * (drivelineefficiency / 100.) * 0.001;
                }
                if (selectedgear[1])
                {
                    maximumvehiclepower = desiredpower * (drivelineefficiency / 100.) * 0.001;
                }


                ImPlot::CreateContext();
                if (ImPlot::BeginPlot("Power vs Vehicle Speed"))
                {
                    ImPlot::SetupAxes("Vehicle Speed (km/h)", "Power (kW)");
                    ImPlot::SetupAxesLimits(0, xaxislimit, 0, yaxislimit);
                    ImPlot::SetNextLineStyle(ImVec4(0.4f, 0.4f, 0.4f, 1.f));
                    ImPlot::PlotInfLines("##vertical line power", &maximumwheelvelocity, 1);
                    ImPlot::SetNextLineStyle(ImVec4(0.4f, 0.4f, 0.4f, 1.f));
                    ImPlot::PlotInfLines("##horizontal line power", &maximumvehiclepower, 1, ImPlotInfLinesFlags_Horizontal);
                    ImPlot::SetNextLineStyle(ImVec4(0.2f, 0.2f, 0.2f, 1.f));
                    ImPlot::PlotLine("Road Resistance", velocitygraph, roadresistance, countroad);
                    for (int i = 0; i < gearnumber; i++)
                    {
                        ImPlot::PlotLine(geartag[i], wheelvelocity[i], powerengine[i], countvelocitylimit[i]);
                    }
                    ImPlot::EndPlot();
                }
                ImPlot::DestroyContext();
            }
        }
        

        ImGui::End();
        ImGui::End();

    }

}
