#include "simulator.hpp"

#include <iostream>
#include <csignal>
#include <cstdlib>
#include <stdexcept>

// ---------------------------------------------------------------------------
// Global simulator pointer for signal handling
// ---------------------------------------------------------------------------
static Simulator* g_sim = nullptr;

static void signal_handler(int sig)
{
    (void)sig;
    std::cout << "\n[main] Caught signal " << sig << " – shutting down.\n";
    if (g_sim) g_sim->stop();
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    std::string events_file = "../data/vertical_line.txt";

    if (argc >= 2) {
        events_file = argv[1];
        std::cout << "[main] Using events file: " << events_file << "\n";
    } else {
        std::cout << "[main] No events file specified, using default: '"
                  << events_file << "'\n";
        std::cout << "       Usage: " << argv[0]
                  << " [events_file.bin]\n\n";
    }

    // Install signal handlers for clean shutdown
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        Simulator sim(events_file);
        g_sim = &sim;
        sim.run();   // blocks until stop() is called
    } catch (const std::exception& ex) {
        std::cerr << "[main] Fatal error: " << ex.what() << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "[main] Simulator exited cleanly.\n";
    return EXIT_SUCCESS;
}
