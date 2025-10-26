using UnityEngine;
using UnityEngine.UI;
using UnityEngine.AI; 
using System.Collections;
using System.Collections.Generic;

[System.Serializable]
public class CameraWaypoint
{
    [Tooltip("Nome descritivo para este ponto.")]
    public string name;

    [Tooltip("O Transform que define a Posição (X,Y,Z) e Rotação FINAIS.")]
    public Transform waypoint;

    [Tooltip("O botão da UI que ativará este ponto.")]
    public Button triggerButton;
}
public class CameraNavMeshController : MonoBehaviour
{
    [Header("Configuração da Câmera")]
    [Tooltip("A câmera que será movimentada.")]
    [SerializeField]
    private Camera targetCamera;

    [Header("Pontos de Navegação (Waypoints)")]
    [SerializeField]
    private List<CameraWaypoint> waypoints = new List<CameraWaypoint>();

    [Header("Configuração da Transição")]
    [Tooltip("A velocidade de movimento da câmera ao longo do caminho.")]
    [SerializeField]
    private float moveSpeed = 5.0f;

    [Tooltip("Duração da rotação inicial quando o alvo está atrás (em segundos).")]
    [SerializeField]
    private float preRotationDuration = 0.5f;

    [Tooltip("Distância máxima para 'projetar' a câmera no NavMesh (deve ser maior que a altura da câmera).")]
    [SerializeField]
    private float navMeshProjectionDistance = 10.0f;

    [Tooltip("Curva de animação para a suavização da velocidade.")]
    [SerializeField]
    private AnimationCurve transitionCurve = AnimationCurve.EaseInOut(0, 0, 1, 1);

    private bool isMoving = false;
    private Coroutine activeMoveCoroutine;
    private NavMeshPath path;

    void Start()
    {
        if (targetCamera == null)
        {
            Debug.LogError("Camera Alvo não definida!", this);
            this.enabled = false;
            return;
        }

        path = new NavMeshPath();
        SetupButtonListeners();
    }

    private void SetupButtonListeners()
    {
        foreach (CameraWaypoint waypoint in waypoints)
        {
            if (waypoint.waypoint == null || waypoint.triggerButton == null)
            {
                Debug.LogWarning($"Waypoint '{waypoint.name}' está incompleto e será ignorado.", this);
                continue;
            }

            CameraWaypoint targetWaypoint = waypoint;
            waypoint.triggerButton.onClick.AddListener(() =>
            {
                TryMoveToWaypoint(targetWaypoint);
            });
        }
    }

    public void TryMoveToWaypoint(CameraWaypoint targetWaypoint)
    {
        if (targetWaypoint == null || targetWaypoint.waypoint == null)
        {
            Debug.LogError("Waypoint de destino é nulo ou inválido.");
            return;
        }

        if (isMoving && activeMoveCoroutine != null)
        {
            StopCoroutine(activeMoveCoroutine);
        }

        Vector3 startPosFloating = targetCamera.transform.position;
        Vector3 endPosFloating = targetWaypoint.waypoint.position;

        bool startFound = NavMesh.SamplePosition(startPosFloating, out NavMeshHit startHit, navMeshProjectionDistance, NavMesh.AllAreas);
        bool endFound = NavMesh.SamplePosition(endPosFloating, out NavMeshHit endHit, navMeshProjectionDistance, NavMesh.AllAreas);

        if (startFound && endFound)
        {
            bool pathFound = NavMesh.CalculatePath(startHit.position, endHit.position, NavMesh.AllAreas, path);

            if (pathFound && path.status == NavMeshPathStatus.PathComplete && path.corners.Length > 1)
            {
                Debug.Log($"NavMeshPath encontrado para '{targetWaypoint.name}'.");
                Vector3 cameraForward = targetCamera.transform.forward;
                Vector3 pathDirection = path.corners[1] - startHit.position;

                cameraForward.y = 0;
                pathDirection.y = 0;

                float dotProduct = Vector3.Dot(cameraForward.normalized, pathDirection.normalized);

                if (dotProduct < -0.1f)
                {
                    Debug.Log("Caminho está atrás. Rotacionando primeiro.");
                    activeMoveCoroutine = StartCoroutine(RotateThenMoveCoroutine(path, targetWaypoint.waypoint));
                }
                else
                {
                    activeMoveCoroutine = StartCoroutine(MoveAlongPathCoroutine(path, targetWaypoint.waypoint));
                }
                return;
            }
        }

        Debug.LogWarning($"Não foi possível encontrar um NavMeshPath para '{targetWaypoint.name}'. Movendo diretamente.", this);
        activeMoveCoroutine = StartCoroutine(MoveDirectlyCoroutine(targetWaypoint.waypoint));
    }

    private IEnumerator RotateThenMoveCoroutine(NavMeshPath path, Transform targetTransform)
    {
        isMoving = true;

        float timeElapsed = 0f;
        Quaternion startRotation = targetCamera.transform.rotation;

        Vector3 directionToLook = path.corners[1] - targetCamera.transform.position;
        directionToLook.y = 0;

        Quaternion targetPreRotation;

        if (directionToLook.sqrMagnitude > 0.001f)
        {
            targetPreRotation = Quaternion.LookRotation(directionToLook.normalized);
        }
        else
        {
            targetPreRotation = startRotation;
        }

        while (timeElapsed < preRotationDuration)
        {
            timeElapsed += Time.deltaTime;
            float t_raw = timeElapsed / preRotationDuration;
            float t_eased = transitionCurve.Evaluate(t_raw);

            targetCamera.transform.rotation = Quaternion.Slerp(startRotation, targetPreRotation, t_eased);
            yield return null;
        }

        targetCamera.transform.rotation = targetPreRotation;

        activeMoveCoroutine = StartCoroutine(MoveAlongPathCoroutine(path, targetTransform));
    }

    private IEnumerator MoveAlongPathCoroutine(NavMeshPath path, Transform targetTransform)
    {
        isMoving = true;

        Vector3[] corners = path.corners;
        if (corners.Length == 0)
        {
            isMoving = false;
            yield break;
        }

        float totalDistance = 0f;
        for (int i = 0; i < corners.Length - 1; i++)
        {
            totalDistance += Vector3.Distance(corners[i], corners[i + 1]);
        }

        if (totalDistance <= 0.01f)
        {
            activeMoveCoroutine = StartCoroutine(MoveDirectlyCoroutine(targetTransform));
            yield break;
        }

        float totalDuration = totalDistance / moveSpeed;
        float timeElapsed = 0f;

        Vector3 startPositionFull = targetCamera.transform.position;
        Vector3 endPositionFull = targetTransform.position;
        Quaternion startRotation = targetCamera.transform.rotation;
        Quaternion endRotation = targetTransform.rotation;

        while (timeElapsed < totalDuration)
        {
            timeElapsed += Time.deltaTime;

            float t_raw = timeElapsed / totalDuration;
            float t_eased = transitionCurve.Evaluate(t_raw);

            float currentDistance = totalDistance * t_eased;
            Vector3 currentFloorPosition = GetPointOnPath(corners, currentDistance);

            float currentY = Mathf.Lerp(startPositionFull.y, endPositionFull.y, t_eased);

            Vector3 newCameraPos = new Vector3(currentFloorPosition.x, currentY, currentFloorPosition.z);

            targetCamera.transform.position = newCameraPos;
            targetCamera.transform.rotation = Quaternion.Slerp(startRotation, endRotation, t_eased);

            yield return null;
        }

        targetCamera.transform.position = endPositionFull;
        targetCamera.transform.rotation = endRotation;
        isMoving = false;
    }
    private IEnumerator MoveDirectlyCoroutine(Transform target)
    {
        isMoving = true;
        float timeElapsed = 0f;

        Vector3 startPosition = targetCamera.transform.position;
        Quaternion startRotation = targetCamera.transform.rotation;
        Vector3 endPosition = target.position;
        Quaternion endRotation = target.rotation;

        float distance = Vector3.Distance(startPosition, endPosition);
        if (distance <= 0.01f)
        {
            targetCamera.transform.position = endPosition;
            targetCamera.transform.rotation = endRotation;
            isMoving = false;
            yield break;
        }

        float duration = distance / moveSpeed;

        while (timeElapsed < duration)
        {
            float t_raw = timeElapsed / duration;
            float t_eased = transitionCurve.Evaluate(t_raw);

            targetCamera.transform.position = Vector3.Lerp(startPosition, endPosition, t_eased);
            targetCamera.transform.rotation = Quaternion.Slerp(startRotation, endRotation, t_eased);

            timeElapsed += Time.deltaTime;
            yield return null;
        }

        targetCamera.transform.position = endPosition;
        targetCamera.transform.rotation = endRotation;
        isMoving = false;
    }
    private Vector3 GetPointOnPath(Vector3[] pathCorners, float distanceToTravel)
    {
        if (pathCorners.Length == 0) return Vector3.zero;
        if (pathCorners.Length == 1) return pathCorners[0];

        float distanceSoFar = 0f;
        for (int i = 0; i < pathCorners.Length - 1; i++)
        {
            Vector3 p0 = pathCorners[i];
            Vector3 p1 = pathCorners[i + 1];
            float segmentDistance = Vector3.Distance(p0, p1);

            if (distanceSoFar + segmentDistance >= distanceToTravel)
            {
                float distanceIntoSegment = distanceToTravel - distanceSoFar;

                if (segmentDistance < 0.001f) return p0;

                float t_segment = distanceIntoSegment / segmentDistance;
                return Vector3.Lerp(p0, p1, t_segment);
            }
            distanceSoFar += segmentDistance;
        }
        return pathCorners[pathCorners.Length - 1];
    }

    void OnDestroy()
    {
        foreach (CameraWaypoint waypoint in waypoints)
        {
            if (waypoint.triggerButton != null)
            {
                waypoint.triggerButton.onClick.RemoveAllListeners();
            }
        }
    }
}