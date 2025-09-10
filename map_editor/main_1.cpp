#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>

struct Node {
    int id;
    cv::Point2f position;
    std::string nodeId;
    
    Node(int _id, cv::Point2f _pos) : id(_id), position(_pos) {
        nodeId = "node_" + std::to_string(_id);
    }
};

struct Edge {
    int id;
    int startNodeId;
    int endNodeId;
    std::string edgeId;
    
    Edge(int _id, int _start, int _end) : id(_id), startNodeId(_start), endNodeId(_end) {
        edgeId = "edge_" + std::to_string(_id);
    }
};

class VDA5050MapEditor {
private:
    cv::Mat canvas;
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    int nextNodeId;
    int nextEdgeId;
    bool isConnectingMode;
    int selectedNodeId;
    
    const int CANVAS_WIDTH = 1200;
    const int CANVAS_HEIGHT = 800;
    const int NODE_RADIUS = 8;
    const cv::Scalar NODE_COLOR = cv::Scalar(0, 255, 0);
    const cv::Scalar EDGE_COLOR = cv::Scalar(255, 0, 0);
    const cv::Scalar SELECTED_COLOR = cv::Scalar(0, 0, 255);

public:
    VDA5050MapEditor() : nextNodeId(1), nextEdgeId(1), isConnectingMode(false), selectedNodeId(-1) {
        canvas = cv::Mat::zeros(CANVAS_HEIGHT, CANVAS_WIDTH, CV_8UC3);
        canvas.setTo(cv::Scalar(255, 255, 255)); // White background
    }
    
    void run() {
        cv::namedWindow("VDA5050 Map Editor", cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback("VDA5050 Map Editor", mouseCallback, this);
        
        showInstructions();
        
        while (true) {
            draw();
            cv::imshow("VDA5050 Map Editor", canvas);
            
            int key = cv::waitKey(30) & 0xFF;
            if (key == 27) break; // ESC to exit
            else if (key == 'c' || key == 'C') toggleConnectMode();
            else if (key == 's' || key == 'S') saveVDA5050Files();
            else if (key == 'r' || key == 'R') reset();
            else if (key == 'h' || key == 'H') showInstructions();
        }
    }
    
    static void mouseCallback(int event, int x, int y, int flags, void* userdata) {
        VDA5050MapEditor* editor = static_cast<VDA5050MapEditor*>(userdata);
        editor->handleMouse(event, x, y, flags);
    }
    
    void handleMouse(int event, int x, int y, int flags) {
        if (event == cv::EVENT_LBUTTONDOWN) {
            if (isConnectingMode) {
                handleConnectionMode(x, y);
            } else {
                addNode(x, y);
            }
        }
    }
    
    void addNode(int x, int y) {
        Node newNode(nextNodeId++, cv::Point2f(x, y));
        nodes.push_back(newNode);
        std::cout << "Added node " << newNode.nodeId << " at (" << x << ", " << y << ")" << std::endl;
    }
    
    void handleConnectionMode(int x, int y) {
        int clickedNodeId = findNearestNode(x, y);
        
        if (clickedNodeId != -1) {
            if (selectedNodeId == -1) {
                selectedNodeId = clickedNodeId;
                std::cout << "Selected node " << nodes[clickedNodeId].nodeId << " for connection" << std::endl;
            } else if (selectedNodeId != clickedNodeId) {
                addEdge(selectedNodeId, clickedNodeId);
                selectedNodeId = -1;
            } else {
                selectedNodeId = -1; // Deselect same node
            }
        }
    }
    
    int findNearestNode(int x, int y) {
        for (size_t i = 0; i < nodes.size(); i++) {
            float dist = cv::norm(nodes[i].position - cv::Point2f(x, y));
            if (dist <= NODE_RADIUS * 2) {
                return i;
            }
        }
        return -1;
    }
    
    void addEdge(int startIdx, int endIdx) {
        Edge newEdge(nextEdgeId++, nodes[startIdx].id, nodes[endIdx].id);
        edges.push_back(newEdge);
        std::cout << "Added edge " << newEdge.edgeId << " from " << nodes[startIdx].nodeId 
                  << " to " << nodes[endIdx].nodeId << std::endl;
    }
    
    void toggleConnectMode() {
        isConnectingMode = !isConnectingMode;
        selectedNodeId = -1;
        std::cout << (isConnectingMode ? "Entered" : "Exited") << " connection mode" << std::endl;
    }
    
    void draw() {
        canvas.setTo(cv::Scalar(255, 255, 255));
        
        // Draw edges
        for (const auto& edge : edges) {
            auto startNode = std::find_if(nodes.begin(), nodes.end(), 
                [&edge](const Node& n) { return n.id == edge.startNodeId; });
            auto endNode = std::find_if(nodes.begin(), nodes.end(), 
                [&edge](const Node& n) { return n.id == edge.endNodeId; });
            
            if (startNode != nodes.end() && endNode != nodes.end()) {
                cv::line(canvas, startNode->position, endNode->position, EDGE_COLOR, 2);
                
                // Draw arrow
                cv::Point2f dir = endNode->position - startNode->position;
                float length = cv::norm(dir);
                if (length > 0) {
                    dir /= length;
                    cv::Point2f arrowHead = endNode->position - dir * 20;
                    cv::Point2f arrow1 = arrowHead + cv::Point2f(-dir.y, dir.x) * 8;
                    cv::Point2f arrow2 = arrowHead + cv::Point2f(dir.y, -dir.x) * 8;
                    
                    cv::line(canvas, endNode->position, arrow1, EDGE_COLOR, 2);
                    cv::line(canvas, endNode->position, arrow2, EDGE_COLOR, 2);
                }
            }
        }
        
        // Draw nodes
        for (size_t i = 0; i < nodes.size(); i++) {
            cv::Scalar color = (selectedNodeId == (int)i) ? SELECTED_COLOR : NODE_COLOR;
            cv::circle(canvas, nodes[i].position, NODE_RADIUS, color, -1);
            
            // Draw node ID
            std::string label = std::to_string(nodes[i].id);
            cv::putText(canvas, label, nodes[i].position + cv::Point2f(12, 5), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        }
        
        // Draw status
        std::string status = "Mode: " + std::string(isConnectingMode ? "Connect" : "Add Node");
        cv::putText(canvas, status, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                   cv::Scalar(0, 0, 0), 2);
        
        if (isConnectingMode && selectedNodeId != -1) {
            std::string selected = "Selected: Node " + std::to_string(nodes[selectedNodeId].id);
            cv::putText(canvas, selected, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                       cv::Scalar(0, 0, 255), 2);
        }
    }
    
    void saveVDA5050Files() {
        saveNodesFile();
        saveEdgesFile();
        saveOrderTopicFile();
        std::cout << "VDA5050 files saved successfully!" << std::endl;
    }
    
    void saveNodesFile() {
        std::ofstream file("vda5050_nodes.json");
        file << "{\n";
        file << "  \"nodes\": [\n";
        
        for (size_t i = 0; i < nodes.size(); i++) {
            file << "    {\n";
            file << "      \"nodeId\": \"" << nodes[i].nodeId << "\",\n";
            file << "      \"x\": " << std::fixed << std::setprecision(2) << nodes[i].position.x / 100.0 << ",\n";
            file << "      \"y\": " << std::fixed << std::setprecision(2) << nodes[i].position.y / 100.0 << ",\n";
            file << "      \"theta\": 0.0,\n";
            file << "      \"allowedDeviationXY\": 0.1,\n";
            file << "      \"allowedDeviationTheta\": 0.1\n";
            file << "    }";
            if (i < nodes.size() - 1) file << ",";
            file << "\n";
        }
        
        file << "  ]\n";
        file << "}\n";
        file.close();
    }
    
    void saveEdgesFile() {
        std::ofstream file("vda5050_edges.json");
        file << "{\n";
        file << "  \"edges\": [\n";
        
        for (size_t i = 0; i < edges.size(); i++) {
            auto startNode = std::find_if(nodes.begin(), nodes.end(), 
                [&edges = edges, i](const Node& n) { return n.id == edges[i].startNodeId; });
            auto endNode = std::find_if(nodes.begin(), nodes.end(), 
                [&edges = edges, i](const Node& n) { return n.id == edges[i].endNodeId; });
                
            file << "    {\n";
            file << "      \"edgeId\": \"" << edges[i].edgeId << "\",\n";
            file << "      \"startNodeId\": \"" << startNode->nodeId << "\",\n";
            file << "      \"endNodeId\": \"" << endNode->nodeId << "\",\n";
            file << "      \"maxSpeed\": 1.0,\n";
            file << "      \"maxHeight\": 2.0,\n";
            file << "      \"minHeight\": 0.0,\n";
            file << "      \"orientation\": 0.0,\n";
            file << "      \"direction\": \"forward\",\n";
            file << "      \"rotationAllowed\": true,\n";
            file << "      \"actions\": []\n";
            file << "    }";
            if (i < edges.size() - 1) file << ",";
            file << "\n";
        }
        
        file << "  ]\n";
        file << "}\n";
        file.close();
    }
    
    void saveOrderTopicFile() {
        std::ofstream file("vda5050_order.json");
        
        // Get current timestamp
        auto now = std::time(nullptr);
        auto tm = *std::localtime(&now);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
        
        file << "{\n";
        file << "  \"headerId\": 1,\n";
        file << "  \"timestamp\": \"" << oss.str() << "\",\n";
        file << "  \"version\": \"2.0.0\",\n";
        file << "  \"manufacturer\": \"MapEditor\",\n";
        file << "  \"serialNumber\": \"12345\",\n";
        file << "  \"orderId\": \"order_" << std::time(nullptr) << "\",\n";
        file << "  \"orderUpdateId\": 1,\n";
        file << "  \"zoneSetId\": \"zone1\",\n";
        file << "  \"nodes\": [\n";
        
        for (size_t i = 0; i < nodes.size(); i++) {
            file << "    {\n";
            file << "      \"nodeId\": \"" << nodes[i].nodeId << "\",\n";
            file << "      \"sequenceId\": " << (i + 1) << ",\n";
            file << "      \"released\": true,\n";
            file << "      \"nodePosition\": {\n";
            file << "        \"x\": " << std::fixed << std::setprecision(2) << nodes[i].position.x / 100.0 << ",\n";
            file << "        \"y\": " << std::fixed << std::setprecision(2) << nodes[i].position.y / 100.0 << ",\n";
            file << "        \"theta\": 0.0,\n";
            file << "        \"allowedDeviationXY\": 0.1,\n";
            file << "        \"allowedDeviationTheta\": 0.1,\n";
            file << "        \"mapId\": \"map1\"\n";
            file << "      },\n";
            file << "      \"actions\": []\n";
            file << "    }";
            if (i < nodes.size() - 1) file << ",";
            file << "\n";
        }
        
        file << "  ],\n";
        file << "  \"edges\": [\n";
        
        for (size_t i = 0; i < edges.size(); i++) {
            auto startNode = std::find_if(nodes.begin(), nodes.end(), 
                [&edges = edges, i](const Node& n) { return n.id == edges[i].startNodeId; });
            auto endNode = std::find_if(nodes.begin(), nodes.end(), 
                [&edges = edges, i](const Node& n) { return n.id == edges[i].endNodeId; });
                
            file << "    {\n";
            file << "      \"edgeId\": \"" << edges[i].edgeId << "\",\n";
            file << "      \"sequenceId\": " << (i + 1) << ",\n";
            file << "      \"released\": true,\n";
            file << "      \"startNodeId\": \"" << startNode->nodeId << "\",\n";
            file << "      \"endNodeId\": \"" << endNode->nodeId << "\",\n";
            file << "      \"actions\": []\n";
            file << "    }";
            if (i < edges.size() - 1) file << ",";
            file << "\n";
        }
        
        file << "  ]\n";
        file << "}\n";
        file.close();
    }
    
    void reset() {
        nodes.clear();
        edges.clear();
        nextNodeId = 1;
        nextEdgeId = 1;
        selectedNodeId = -1;
        isConnectingMode = false;
        std::cout << "Map reset!" << std::endl;
    }
    
    void showInstructions() {
        std::cout << "\n=== VDA5050 Map Editor Instructions ===" << std::endl;
        std::cout << "Left Click: Add node (in add mode) / Select nodes (in connect mode)" << std::endl;
        std::cout << "C: Toggle connect mode" << std::endl;
        std::cout << "S: Save VDA5050 files" << std::endl;
        std::cout << "R: Reset map" << std::endl;
        std::cout << "H: Show instructions" << std::endl;
        std::cout << "ESC: Exit" << std::endl;
        std::cout << "======================================\n" << std::endl;
    }
};

int main() {
    try {
        VDA5050MapEditor editor;
        editor.run();
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV error: " << e.what() << std::endl;
        return -1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}