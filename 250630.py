import json

def load_geojson(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        return json.load(f)

def save_geojson(data, filepath):
    with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

def extract_id_pairs_from_A(features_A):
    # A에서 NODE_ID, NODE_MAP_I를 정수로 추출
    return set(
        (feature['properties'].get('NODE_ID'), feature['properties'].get('NODE_MAP_I'))
        for feature in features_A
        if isinstance(feature['properties'].get('NODE_ID'), int) and isinstance(feature['properties'].get('NODE_MAP_I'), int)
    )

def filter_B_by_A(B_features, id_pair_set):
    # B에서 정수로 변환 후 A에 있는 항목만 필터링
    filtered = []
    for feature in B_features:
        props = feature.get('properties', {})
        try:
            node_id = int(props.get('NODE_ID'))
            node_map_i = int(props.get('NODE_MAP_I'))
        except (TypeError, ValueError):
            continue  # 변환 불가시 무시

        if (node_id, node_map_i) in id_pair_set:
            filtered.append(feature)
    return filtered

def main():
    A_geojson = load_geojson('A.geojson')
    B_geojson = load_geojson('B.geojson')

    id_pairs = extract_id_pairs_from_A(A_geojson['features'])
    filtered_B_features = filter_B_by_A(B_geojson['features'], id_pairs)

    result = {
        "type": "FeatureCollection",
        "features": filtered_B_features
    }

    save_geojson(result, 'B_filtered.geojson')
    print(f"필터링된 항목 개수: {len(filtered_B_features)}개")

if __name__ == '__main__':
    main()