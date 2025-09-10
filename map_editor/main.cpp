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
#include <cmath>

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
    std::string type; // "linear" or "arc"
    cv::Point2f arcCenter; // Only for arc edges
    
    Edge(int _id, int _start, int _end, std::string _type = "linear") 
        : id(_id), startNodeId(_start), endNodeId(_end), type(_type), arcCenter(0, 0) {
        edgeId = "edge_" + std::to_string(_id);
    }
    
    Edge(int _id, int _start, int _end, cv::Point2f _arcCenter) 
        : id(_id), startNodeId(_start), endNodeId(_end), type("arc"), arcCenter(_arcCenter) {
        edgeId = "edge_" + std::to_string(_id);
    }
};

enum class EditMode {
    ADD_NODE,
    CONNECT,
    EDIT,
    DELETE
};

class VDA5050MapEditor {
private:
    cv::Mat canvas;
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    int nextNodeId;
    int nextEdgeId;
    EditMode currentMode;
    int selectedNodeId;
    int selectedEdgeId;
    std::vector<int> selectedEdgesForArc;
    
    // Edit UI variables
    bool showEditUI;
    cv::Point2f editUIPosition;
    
    const int CANVAS_WIDTH = 1200;
    const int CANVAS_HEIGHT = 800;
    const int NODE_RADIUS = 8;
    const int UI_PANEL_WIDTH = 300;
    const cv::Scalar NODE_COLOR = cv::Scalar(0, 255, 0);
    const cv::Scalar EDGE_COLOR = cv::Scalar(255, 0, 0);
    const cv::Scalar ARC_EDGE_COLOR = cv::Scalar(255, 165, 0);
    const cv::Scalar SELECTED_COLOR = cv::Scalar(0, 0, 255);
    const cv::Scalar DELETE_COLOR = cv::Scalar(0, 0, 128);

public:
    VDA5050MapEditor() : nextNodeId(1), nextEdgeId(1), currentMode(EditMode::ADD_NODE), 
                         selectedNodeId(-1), selectedEdgeId(-1), showEditUI(false) {
        canvas = cv::Mat::zeros(CANVAS_HEIGHT, CANVAS_WIDTH + UI_PANEL_WIDTH, CV_8UC3);
        canvas.setTo(cv::Scalar(255, 255, 255));
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
            else if (key == 'c' || key == 'C') setMode(EditMode::CONNECT);
            else if (key == 'e' || key == 'E') setMode(EditMode::EDIT);
            else if (key == 'd' || key == 'D') setMode(EditMode::DELETE);
            else if (key == 'a' || key == 'A') setMode(EditMode::ADD_NODE);
            else if (key == 's' || key == 'S') saveVDA5050Files();
            else if (key == 'r' || key == 'R') reset();
            else if (key == 'h' || key == 'H') showInstructions();
            else if (key == 'z' || key == 'Z') createArcFromSelectedEdges();
            else if (key == 13) applyEdit(); // Enter key
        }
    }
    
    static void mouseCallback(int event, int x, int y, int flags, void* userdata) {
        VDA5050MapEditor* editor = static_cast<VDA5050MapEditor*>(userdata);
        editor->handleMouse(event, x, y, flags);
    }
    
    void handleMouse(int event, int x, int y, int flags) {
        if (x >= CANVAS_WIDTH) return; // Ignore clicks in UI panel
        
        if (event == cv::EVENT_LBUTTONDOWN) {
            switch (currentMode) {
                case EditMode::ADD_NODE:
                    addNode(x, y);
                    break;
                case EditMode::CONNECT:
                    handleConnectionMode(x, y);
                    break;
                case EditMode::EDIT:
                    handleEditMode(x, y);
                    break;
                case EditMode::DELETE:
                    handleDeleteMode(x, y);
                    break;
            }
        }
    }
    
    void setMode(EditMode mode) {
        currentMode = mode;
        selectedNodeId = -1;
        selectedEdgeId = -1;
        selectedEdgesForArc.clear();
        showEditUI = false;
        
        std::string modeStr;
        switch (mode) {
            case EditMode::ADD_NODE: modeStr = "Add Node"; break;
            case EditMode::CONNECT: modeStr = "Connect"; break;
            case EditMode::EDIT: modeStr = "Edit"; break;
            case EditMode::DELETE: modeStr = "Delete"; break;
        }
        std::cout << "Mode changed to: " << modeStr << std::endl;
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
                selectedNodeId = -1;
            }
        }
    }
    
    void handleEditMode(int x, int y) {
        int clickedNodeId = findNearestNode(x, y);
        int clickedEdgeId = findNearestEdge(x, y);
        
        if (clickedNodeId != -1) {
            selectedNodeId = clickedNodeId;
            selectedEdgeId = -1;
            showNodeEditUI(clickedNodeId, x, y);
        } else if (clickedEdgeId != -1) {
            selectedEdgeId = clickedEdgeId;
            selectedNodeId = -1;
            showEdgeEditUI(clickedEdgeId, x, y);
        }
    }
    
    void handleDeleteMode(int x, int y) {
        int clickedNodeId = findNearestNode(x, y);
        int clickedEdgeId = findNearestEdge(x, y);
        
        if (clickedNodeId != -1) {
            deleteNode(clickedNodeId);
        } else if (clickedEdgeId != -1) {
            deleteEdge(clickedEdgeId);
        }
    }
    
    void deleteNode(int nodeIdx) {
        int nodeId = nodes[nodeIdx].id;
        
        // Remove all edges connected to this node
        edges.erase(std::remove_if(edges.begin(), edges.end(),
            [nodeId](const Edge& e) { 
                return e.startNodeId == nodeId || e.endNodeId == nodeId; 
            }), edges.end());
        
        std::cout << "Deleted node " << nodes[nodeIdx].nodeId << " and connected edges" << std::endl;
        nodes.erase(nodes.begin() + nodeIdx);
    }
    
    void deleteEdge(int edgeIdx) {
        std::cout << "Deleted edge " << edges[edgeIdx].edgeId << std::endl;
        edges.erase(edges.begin() + edgeIdx);
    }
    
    void showNodeEditUI(int nodeIdx, int x, int y) {
        showEditUI = true;
        editUIPosition = cv::Point2f(x, y);
        std::cout << "Editing node " << nodes[nodeIdx].nodeId 
                  << " at (" << nodes[nodeIdx].position.x << ", " << nodes[nodeIdx].position.y << ")" << std::endl;
        std::cout << "Use arrow keys to move, Enter to apply" << std::endl;
    }
    
    void showEdgeEditUI(int edgeIdx, int x, int y) {
        showEditUI = true;
        editUIPosition = cv::Point2f(x, y);
        
        auto startNode = std::find_if(nodes.begin(), nodes.end(),
            [&edges = edges, edgeIdx](const Node& n) { return n.id == edges[edgeIdx].startNodeId; });
        auto endNode = std::find_if(nodes.begin(), nodes.end(),
            [&edges = edges, edgeIdx](const Node& n) { return n.id == edges[edgeIdx].endNodeId; });
            
        std::cout << "Editing edge " << edges[edgeIdx].edgeId;
        if (startNode != nodes.end() && endNode != nodes.end()) {
            std::cout << " from " << startNode->nodeId << " to " << endNode->nodeId;
        }
        std::cout << std::endl;
        
        // Check if this edge can be selected for arc creation
        auto it = std::find(selectedEdgesForArc.begin(), selectedEdgesForArc.end(), edgeIdx);
        if (it == selectedEdgesForArc.end() && selectedEdgesForArc.size() < 2) {
            selectedEdgesForArc.push_back(edgeIdx);
            std::cout << "Edge selected for arc creation (" << selectedEdgesForArc.size() << "/2)" << std::endl;
        }
    }
    
    void applyEdit() {
        if (selectedNodeId != -1) {
            // Node editing with arrow keys could be implemented here
            std::cout << "Node edit applied" << std::endl;
        }
        showEditUI = false;
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
    
    int findNearestEdge(int x, int y) {
        cv::Point2f clickPoint(x, y);
        const float EDGE_THRESHOLD = 10.0f;
        
        for (size_t i = 0; i < edges.size(); i++) {
            auto startNode = std::find_if(nodes.begin(), nodes.end(),
                [&edges = edges, i](const Node& n) { return n.id == edges[i].startNodeId; });
            auto endNode = std::find_if(nodes.begin(), nodes.end(),
                [&edges = edges, i](const Node& n) { return n.id == edges[i].endNodeId; });
                
            if (startNode != nodes.end() && endNode != nodes.end()) {
                if (edges[i].type == "linear") {
                    // Check distance to line segment
                    float dist = distanceToLineSegment(clickPoint, startNode->position, endNode->position);
                    if (dist <= EDGE_THRESHOLD) {
                        return i;
                    }
                } else if (edges[i].type == "arc") {
                    // Check distance to arc
                    float dist = distanceToArc(clickPoint, startNode->position, endNode->position, edges[i].arcCenter);
                    if (dist <= EDGE_THRESHOLD) {
                        return i;
                    }
                }
            }
        }
        return -1;
    }
    
    float distanceToLineSegment(cv::Point2f p, cv::Point2f a, cv::Point2f b) {
        cv::Point2f ab = b - a;
        cv::Point2f ap = p - a;
        float ab2 = ab.dot(ab);
        float ap_ab = ap.dot(ab);
        float t = std::max(0.0f, std::min(1.0f, ap_ab / ab2));
        cv::Point2f projection = a + t * ab;
        return cv::norm(p - projection);
    }
    
    float distanceToArc(cv::Point2f p, cv::Point2f start, cv::Point2f end, cv::Point2f center) {
        // Simplified arc distance calculation
        float distToCenter = cv::norm(p - center);
        float radius = cv::norm(start - center);
        return std::abs(distToCenter - radius);
    }
    
    void createArcFromSelectedEdges() {
        if (selectedEdgesForArc.size() != 2) {
            std::cout << "Please select exactly 2 edges for arc creation" << std::endl;
            return;
        }
        
        int edge1Idx = selectedEdgesForArc[0];
        int edge2Idx = selectedEdgesForArc[1];
        
        // Check if edges are connected
        Edge& edge1 = edges[edge1Idx];
        Edge& edge2 = edges[edge2Idx];
        
        int commonNodeId = -1;
        if (edge1.endNodeId == edge2.startNodeId) {
            commonNodeId = edge1.endNodeId;
        } else if (edge1.startNodeId == edge2.endNodeId) {
            commonNodeId = edge1.startNodeId;
        } else {
            std::cout << "Selected edges are not connected" << std::endl;
            selectedEdgesForArc.clear();
            return;
        }
        
        // Find the common node
        auto commonNode = std::find_if(nodes.begin(), nodes.end(),
            [commonNodeId](const Node& n) { return n.id == commonNodeId; });
            
        if (commonNode == nodes.end()) {
            std::cout << "Common node not found" << std::endl;
            selectedEdgesForArc.clear();
            return;
        }
        
        // Get the other endpoints
        auto node1 = std::find_if(nodes.begin(), nodes.end(),
            [&edge1, commonNodeId](const Node& n) { 
                return n.id == (edge1.startNodeId == commonNodeId ? edge1.endNodeId : edge1.startNodeId); 
            });
        auto node2 = std::find_if(nodes.begin(), nodes.end(),
            [&edge2, commonNodeId](const Node& n) { 
                return n.id == (edge2.startNodeId == commonNodeId ? edge2.endNodeId : edge2.startNodeId); 
            });
            
        if (node1 == nodes.end() || node2 == nodes.end()) {
            std::cout << "Endpoint nodes not found" << std::endl;
            selectedEdgesForArc.clear();
            return;
        }
        
        // Calculate arc center (simplified - perpendicular bisector intersection)
        cv::Point2f arcCenter = calculateArcCenter(node1->position, commonNode->position, node2->position);
        
        // Create new intermediate node at arc center projection
        cv::Point2f midPoint = (node1->position + node2->position) * 0.5f;
        Node arcNode(nextNodeId++, midPoint);
        nodes.push_back(arcNode);
        
        // Create three new edges
        Edge newEdge1(nextEdgeId++, node1->id, arcNode.id, "linear");
        Edge arcEdge(nextEdgeId++, arcNode.id, arcNode.id, arcCenter); // Arc edge
        arcEdge.type = "arc";
        arcEdge.startNodeId = node1->id;
        arcEdge.endNodeId = node2->id;
        Edge newEdge2(nextEdgeId++, arcNode.id, node2->id, "linear");
        
        // Remove old edges (in reverse order to maintain indices)
        if (edge1Idx > edge2Idx) {
            edges.erase(edges.begin() + edge1Idx);
            edges.erase(edges.begin() + edge2Idx);
        } else {
            edges.erase(edges.begin() + edge2Idx);
            edges.erase(edges.begin() + edge1Idx);
        }
        
        // Add new edges
        edges.push_back(newEdge1);
        edges.push_back(arcEdge);
        edges.push_back(newEdge2);
        
        std::cout << "Created arc from selected edges with center at (" 
                  << arcCenter.x << ", " << arcCenter.y << ")" << std::endl;
        
        selectedEdgesForArc.clear();
    }
    
    // cv::Point2f calculateArcCenter(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3) {
        // // Calculate arc center using perpendicular bisectors
        // // Simplified calculation - you might want to implement a more robust method
        // cv::Point2f mid12 = (p1 + p2) * 0.5f;
        // cv::Point2f mid23 = (p2 + p3) * 0.5f;
        
        // cv::Point2f dir12 = p2 - p1;
        // cv::Point2f perp12(-dir12.y, dir12.x);
        
        // cv::Point2f dir23 = p3 - p2;
        // cv::Point2f perp23(-dir23.y, dir23.x);
        
        // // Find intersection of perpendicular bisectors
        // // For simplicity, return a point offset from the middle node
        // cv::Point2f offset = cv::Point2f(30, 30); // Fixed offset for demo
        // return p2 + offset;

cv::Point2f calculateArcCenter(cv::Point2f A, cv::Point2f B, cv::Point2f C) {
    float a1 = B.x - A.x, b1 = B.y - A.y;
    float a2 = C.x - B.x, b2 = C.y - B.y;

    float mid1x = (A.x + B.x) / 2.0f, mid1y = (A.y + B.y) / 2.0f;
    float mid2x = (B.x + C.x) / 2.0f, mid2y = (B.y + C.y) / 2.0f;

    float d = a1 * b2 - a2 * b1;
    if (fabs(d) < 1e-6) return B; // 직선인 경우 예외 처리

    float t = ((mid2x - mid1x) * b2 - (mid2y - mid1y) * a2) / d;
    return cv::Point2f(mid1x + t * a1, mid1y + t * b1);

    }
    
    void addEdge(int startIdx, int endIdx) {
        Edge newEdge(nextEdgeId++, nodes[startIdx].id, nodes[endIdx].id);
        edges.push_back(newEdge);
        std::cout << "Added edge " << newEdge.edgeId << " from " << nodes[startIdx].nodeId 
                  << " to " << nodes[endIdx].nodeId << std::endl;
    }
    
    void draw() {
        canvas.setTo(cv::Scalar(255, 255, 255));
        
        // Draw main canvas area
        cv::rectangle(canvas, cv::Point(0, 0), cv::Point(CANVAS_WIDTH, CANVAS_HEIGHT), 
                     cv::Scalar(240, 240, 240), 1);
        
        // Draw edges
        for (size_t i = 0; i < edges.size(); i++) {
            drawEdge(i);
        }
        
        // Draw nodes
        for (size_t i = 0; i < nodes.size(); i++) {
            cv::Scalar color = NODE_COLOR;
            if (currentMode == EditMode::DELETE) color = DELETE_COLOR;
            if ((int)i == selectedNodeId) color = SELECTED_COLOR;
            
            cv::circle(canvas, nodes[i].position, NODE_RADIUS, color, -1);
            
            // Draw node ID
            std::string label = std::to_string(nodes[i].id);
            cv::putText(canvas, label, nodes[i].position + cv::Point2f(12, 5), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        }
        
        // Draw UI panel
        drawUIPanel();
        
        // Draw edit UI if active
        if (showEditUI) {
            drawEditUI();
        }
    }
    
    void drawEdge(int edgeIdx) {
        const Edge& edge = edges[edgeIdx];
        auto startNode = std::find_if(nodes.begin(), nodes.end(),
            [&edge](const Node& n) { return n.id == edge.startNodeId; });
        auto endNode = std::find_if(nodes.begin(), nodes.end(),
            [&edge](const Node& n) { return n.id == edge.endNodeId; });
            
        if (startNode == nodes.end() || endNode == nodes.end()) return;
        
        cv::Scalar color = (edge.type == "arc") ? ARC_EDGE_COLOR : EDGE_COLOR;
        if (currentMode == EditMode::DELETE) color = DELETE_COLOR;
        if (edgeIdx == selectedEdgeId) color = SELECTED_COLOR;
        
        // Check if this edge is selected for arc creation
        if (std::find(selectedEdgesForArc.begin(), selectedEdgesForArc.end(), edgeIdx) != selectedEdgesForArc.end()) {
            color = cv::Scalar(128, 0, 128); // Purple for selected edges
        }
        
        if (edge.type == "linear") {
            cv::line(canvas, startNode->position, endNode->position, color, 2);
            drawArrow(startNode->position, endNode->position, color);
        } else if (edge.type == "arc") {
            drawArc(startNode->position, endNode->position, edge.arcCenter, color);
        }
    }
    
    void drawArrow(cv::Point2f start, cv::Point2f end, cv::Scalar color) {
        cv::Point2f dir = end - start;
        float length = cv::norm(dir);
        if (length > 0) {
            dir /= length;
            cv::Point2f arrowHead = end - dir * 20;
            cv::Point2f arrow1 = arrowHead + cv::Point2f(-dir.y, dir.x) * 8;
            cv::Point2f arrow2 = arrowHead + cv::Point2f(dir.y, -dir.x) * 8;
            
            cv::line(canvas, end, arrow1, color, 2);
            cv::line(canvas, end, arrow2, color, 2);
        }
    }
    
    void drawArc(cv::Point2f start, cv::Point2f end, cv::Point2f center, cv::Scalar color) {
        float radius = cv::norm(start - center);
        float startAngle = atan2(start.y - center.y, start.x - center.x);
        float endAngle = atan2(end.y - center.y, end.x - center.x);
        
        // Draw arc using ellipse
        cv::ellipse(canvas, center, cv::Size(radius, radius), 0, 
                   startAngle * 180.0 / CV_PI, endAngle * 180.0 / CV_PI, color, 2);
        
        // Draw center point
        cv::circle(canvas, center, 3, color, -1);
        
        // Draw arrow at end
        cv::Point2f tangent(-sin(endAngle), cos(endAngle));
        cv::Point2f arrowStart = end - tangent * 15;
        cv::Point2f arrow1 = arrowStart + cv::Point2f(-tangent.y, tangent.x) * 8;
        cv::Point2f arrow2 = arrowStart + cv::Point2f(tangent.y, -tangent.x) * 8;
        
        cv::line(canvas, end, arrow1, color, 2);
        cv::line(canvas, end, arrow2, color, 2);
    }
    
    void drawUIPanel() {
        // Draw UI panel background
        cv::rectangle(canvas, cv::Point(CANVAS_WIDTH, 0), 
                     cv::Point(CANVAS_WIDTH + UI_PANEL_WIDTH, CANVAS_HEIGHT), 
                     cv::Scalar(250, 250, 250), -1);
        cv::rectangle(canvas, cv::Point(CANVAS_WIDTH, 0), 
                     cv::Point(CANVAS_WIDTH + UI_PANEL_WIDTH, CANVAS_HEIGHT), 
                     cv::Scalar(200, 200, 200), 2);
        
        int yPos = 30;
        int lineHeight = 25;
        
        // Current mode
        std::string modeStr;
        switch (currentMode) {
            case EditMode::ADD_NODE: modeStr = "Add Node"; break;
            case EditMode::CONNECT: modeStr = "Connect"; break;
            case EditMode::EDIT: modeStr = "Edit"; break;
            case EditMode::DELETE: modeStr = "Delete"; break;
        }
        cv::putText(canvas, "Mode: " + modeStr, cv::Point(CANVAS_WIDTH + 10, yPos), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
        yPos += lineHeight * 2;
        
        // Statistics
        cv::putText(canvas, "Nodes: " + std::to_string(nodes.size()), 
                   cv::Point(CANVAS_WIDTH + 10, yPos), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                   cv::Scalar(0, 0, 0), 1);
        yPos += lineHeight;
        
        cv::putText(canvas, "Edges: " + std::to_string(edges.size()), 
                   cv::Point(CANVAS_WIDTH + 10, yPos), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                   cv::Scalar(0, 0, 0), 1);
        yPos += lineHeight * 2;
        
        // Controls
        cv::putText(canvas, "Controls:", cv::Point(CANVAS_WIDTH + 10, yPos), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
        yPos += lineHeight;
        
        cv::putText(canvas, "A - Add Node", cv::Point(CANVAS_WIDTH + 10, yPos), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
        yPos += lineHeight;
        
        cv::putText(canvas, "C - Connect", cv::Point(CANVAS_WIDTH + 10, yPos), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
        yPos += lineHeight;
        
        cv::putText(canvas, "E - Edit", cv::Point(CANVAS_WIDTH + 10, yPos), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
        yPos += lineHeight;
        
        cv::putText(canvas, "D - Delete", cv::Point(CANVAS_WIDTH + 10, yPos), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
        yPos += lineHeight;
        
        cv::putText(canvas, "Z - Create Arc", cv::Point(CANVAS_WIDTH + 10, yPos), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
        yPos += lineHeight;
        
        cv::putText(canvas, "S - Save", cv::Point(CANVAS_WIDTH + 10, yPos), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
        yPos += lineHeight;
        
        // Selected edges for arc
        if (!selectedEdgesForArc.empty()) {
            yPos += lineHeight;
            cv::putText(canvas, "Arc Creation:", cv::Point(CANVAS_WIDTH + 10, yPos), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(128, 0, 128), 1);
            yPos += lineHeight;
            cv::putText(canvas, std::to_string(selectedEdgesForArc.size()) + "/2 edges selected", 
                       cv::Point(CANVAS_WIDTH + 10, yPos), cv::FONT_HERSHEY_SIMPLEX, 0.4, 
                       cv::Scalar(128, 0, 128), 1);
        }
    }
    
    void drawEditUI() {
        // Simple edit indicator
        cv::circle(canvas, editUIPosition, 15, cv::Scalar(0, 255, 255), 2);
        cv::putText(canvas, "EDIT", editUIPosition + cv::Point2f(-15, -20), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);
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
            file << "      \"type\": \"" << edges[i].type << "\",\n";
            
            if (edges[i].type == "arc") {
                file << "      \"arcCenter\": {\n";
                file << "        \"x\": " << std::fixed << std::setprecision(2) << edges[i].arcCenter.x / 100.0 << ",\n";
                file << "        \"y\": " << std::fixed << std::setprecision(2) << edges[i].arcCenter.y / 100.0 << "\n";
                file << "      },\n";
            }
            
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
            file << "      \"type\": \"" << edges[i].type << "\",\n";
            
            if (edges[i].type == "arc") {
                file << "      \"arcCenter\": {\n";
                file << "        \"x\": " << std::fixed << std::setprecision(2) << edges[i].arcCenter.x / 100.0 << ",\n";
                file << "        \"y\": " << std::fixed << std::setprecision(2) << edges[i].arcCenter.y / 100.0 << "\n";
                file << "      },\n";
            }
            
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
        selectedEdgeId = -1;
        selectedEdgesForArc.clear();
        showEditUI = false;
        currentMode = EditMode::ADD_NODE;
        std::cout << "Map reset!" << std::endl;
    }
    
    void showInstructions() {
        std::cout << "\n=== VDA5050 Map Editor Instructions ===" << std::endl;
        std::cout << "Left Click: Add node / Select for connection/edit/delete" << std::endl;
        std::cout << "A: Add Node mode" << std::endl;
        std::cout << "C: Connect mode" << std::endl;
        std::cout << "E: Edit mode (select edges for arc creation)" << std::endl;
        std::cout << "D: Delete mode" << std::endl;
        std::cout << "Z: Create arc from 2 selected edges" << std::endl;
        std::cout << "S: Save VDA5050 files" << std::endl;
        std::cout << "R: Reset map" << std::endl;
        std::cout << "H: Show instructions" << std::endl;
        std::cout << "ESC: Exit" << std::endl;
        std::cout << "\n--- Arc Creation ---" << std::endl;
        std::cout << "1. Enter Edit mode (E)" << std::endl;
        std::cout << "2. Click on 2 connected edges" << std::endl;
        std::cout << "3. Press Z to create arc" << std::endl;
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